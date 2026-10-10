//! Scan config.
use core::future::Future;

use bt_hci::cmd::le::{
    LeAddDeviceToFilterAcceptList,
    LeAddDeviceToPeriodicAdvList,
    LeClearFilterAcceptList,
    LeClearPeriodicAdvList,
    LePeriodicAdvCreateSync,
    LePeriodicAdvCreateSyncParams,
    LeSetExtScanEnable,
    LeSetExtScanParams,
    //NEW ADDED
    // LE Set Periodic Advertising Receive Enable command [📖](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host-controller-interface/host-controller-interface-functional-specification.html#UUID-b055b724-7607-bf63-3862-3e164dfc2251)
    //and param
    LeSetPeriodicAdvReceiveEnable,
    LeSetPeriodicAdvReceiveEnableParams,
    LeSetScanEnable,
    LeSetScanParams,
};
use bt_hci::controller::{Controller, ControllerCmdAsync, ControllerCmdSync};
use bt_hci::param::{AddrKind, FilterDuplicates, ScanningPhy};
pub use bt_hci::param::{LeAdvReportsIter, LeExtAdvReportsIter};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;

use crate::command::CommandState;
use crate::connection::ScanConfig;
use crate::{bt_hci_duration, BleHostError, Central, PacketPool};

/// A scanner that wraps a central to provide additional functionality
/// around BLE scanning.
///
/// The buffer size can be tuned if in a noisy environment that
/// returns a lot of results.
pub struct Scanner<'d, 'stack, C: Controller, P: PacketPool> {
    central: &'d mut Central<'stack, C, P>,
}

impl<'d, 'stack, C: Controller, P: PacketPool> Scanner<'d, 'stack, C, P> {
    /// Create a new scanner with the provided central.
    pub fn new(central: &'d mut Central<'stack, C, P>) -> Self {
        Self { central }
    }

    /// /// Retrieve the underlying central
    /// pub fn into_inner(self) -> Central<'d, C, P> {
    ///     self.central
    /// }

    /// Enable extended scanning and synchronize with a periodic advertising train from an advertiser.
    ///
    /// Details on the underlying command can be found here:
    /// https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host-controller-interface/host-controller-interface-functional-specification.html#UUID-29188ef0-bf80-7807-2c96-385e7d9782ed
    pub async fn scan_periodic(
        &mut self,
        config: &ScanConfig<'_>,
        param: LePeriodicAdvCreateSyncParams,
    ) -> Result<ScanSession<'_, true>, BleHostError<C::Error>>
    where
        C: ControllerCmdAsync<LePeriodicAdvCreateSync> //Note the Async variation here
            + ControllerCmdSync<LeClearPeriodicAdvList>
            + ControllerCmdSync<LeAddDeviceToPeriodicAdvList>
            + ControllerCmdSync<LeSetPeriodicAdvReceiveEnable>
            + ControllerCmdSync<LeSetExtScanEnable> //We also enable extended scanning
            + ControllerCmdSync<LeSetExtScanParams>
            + ControllerCmdSync<LeClearFilterAcceptList>
            + ControllerCmdSync<LeAddDeviceToFilterAcceptList>,
    {
        //First we enable extended scanning.
        let host = self.central.host;
        let drop = crate::host::OnDrop::new(|| {
            host.scan_command_state().cancel(true); //I think this should be true
        });

        host.request_operation(host.scan_command_state(), true).await;
        // NOTE: Maybe this causes an error
        //self.central.set_accept_filter(config.filter_accept_list).await?; --maybe this causes an error

        let scanning = ScanningPhy {
            active_scan: config.active,
            scan_interval: bt_hci_duration(config.interval),
            scan_window: bt_hci_duration(config.window),
        };

        let phy_params = crate::central::create_phy_params(scanning, config.phys);
        let host = &self.central.host;

        //maybe comment this out to fall back on Vendor default
        host.command(LeSetExtScanParams::new(
            host.own_addr_kind(),
            if config.filter_accept_list.is_empty() {
                bt_hci::param::ScanningFilterPolicy::BasicUnfiltered
            } else {
                bt_hci::param::ScanningFilterPolicy::BasicFiltered
            },
            phy_params,
        ))
        .await?;

        //This is the difference from simply scan_ext --it is fine to put this before ext scan enable
        // NOTE: This might outlive extended scanning, and might need to be properly canceled or
        // more expicitly be given a lifetime, such that the user knows to shut this down, before
        // opening a new one (as doing so would give an HCI invalid or faulty command smth smth.).
        host.async_command(LePeriodicAdvCreateSync::new(
            param.options,
            param.adv_sid,
            param.adv_addr_kind,
            param.adv_addr,
            param.skip,
            param.sync_timeout,
            param.sync_cte_kind,
        ))
        .await?;

        //config.window is the duration of the scan.
        //config.interval is the period of the scan.
        //We can scan continuously by settting these to 0 seconds.
        host.command(LeSetExtScanEnable::new(
            true,
            config.filter_duplicates,
            bt_hci_duration(config.timeout),
            bt_hci::param::Duration::from_secs(0),
        ))
        .await?;

        //This is the difference from simply scan_ext
        // host.async_command(LePeriodicAdvCreateSync::new(
        //     param.options,
        //     param.adv_sid,
        //     param.adv_addr_kind,
        //     param.adv_addr,
        //     param.skip,
        //     param.sync_timeout,
        //     param.sync_cte_kind
        // ))
        // .await?;

        // embassy_time::Timer::after_secs(5).await;
        // //This is indeed a sync command--
        // HACK: we know the SyncHandle will be 0
        host.command(LeSetPeriodicAdvReceiveEnable::new(
            bt_hci::param::SyncHandle::default(),
            bt_hci::param::LePeriodicAdvReceiveEnable::new()
                .set_duplicate_filtering(false)
                .set_reporting(true),
        ))
        .await?;

        drop.defuse();
        Ok(ScanSession {
            command_state: &self.central.host.scan_command_state(),
            deadline: if config.timeout.as_ticks() == 0 {
                None
            } else {
                Some(Instant::now() + config.timeout)
            },
            done: false,
            timeout: host.scan_timeout(),
        })
    }

    /// Performs an extended BLE scan, return a report for discovering peripherals.
    ///
    /// Scan is stopped when a report is received -- *Is it really??*. Call this method repeatedly to continue scanning.
    pub async fn scan_ext(&mut self, config: &ScanConfig<'_>) -> Result<ScanSession<'_, true>, BleHostError<C::Error>>
    where
        C: ControllerCmdSync<LeSetExtScanEnable>
            + ControllerCmdSync<LeSetExtScanParams>
            + ControllerCmdSync<LeClearFilterAcceptList>
            + ControllerCmdSync<LeAddDeviceToFilterAcceptList>,
    {
        let host = self.central.host;
        let drop = crate::host::OnDrop::new(|| {
            host.scan_command_state().cancel(true);
        });
        host.request_operation(host.scan_command_state(), true).await;
        self.central.set_accept_filter(config.filter_accept_list).await?;

        let scanning = ScanningPhy {
            active_scan: config.active,
            scan_interval: bt_hci_duration(config.interval),
            scan_window: bt_hci_duration(config.window),
        };
        let phy_params = crate::central::create_phy_params(scanning, config.phys);
        let host = &self.central.host;
        host.command(LeSetExtScanParams::new(
            host.own_addr_kind(),
            if config.filter_accept_list.is_empty() {
                bt_hci::param::ScanningFilterPolicy::BasicUnfiltered
            } else {
                bt_hci::param::ScanningFilterPolicy::BasicFiltered
            },
            phy_params,
        ))
        .await?;

        host.scan_timeout().reset();
        host.command(LeSetExtScanEnable::new(
            true,
            config.filter_duplicates,
            bt_hci_duration(config.timeout),
            bt_hci::param::Duration::from_secs(0),
        ))
        .await?;
        drop.defuse();
        Ok(ScanSession {
            command_state: self.central.host.scan_command_state(),
            deadline: if config.timeout.as_ticks() == 0 {
                None
            } else {
                Some(Instant::now() + config.timeout)
            },
            timeout: host.scan_timeout(),
            done: false,
        })
    }

    ///Stop extended scanning. Needed as currently the timeout for ScanSession does nothing.
    /// Disabling scanning when it is already disabled has no effect.
    pub async fn stop_ext_scan(&mut self) -> Result<(), BleHostError<C::Error>>
    where
        C: ControllerCmdSync<LeSetExtScanEnable>,
    {
        let host = &self.central.host;

        //The Enable parameter determines whether scanning is enabled or disabled.
        //If it is set to 0x00, the remaining parameters shall be ignored.
        //https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host-controller-interface/host-controller-interface-functional-specification.html#UUID-bfe8407c-4def-2ded-51dd-e47cf9e8916c:~:text=7.8.65.%20LE%20Set%20Extended%20Scan%20Enable%20command
        host.command(LeSetExtScanEnable::new(
            false,
            FilterDuplicates::Disabled,
            bt_hci::param::Duration::from_secs(0),
            bt_hci::param::Duration::from_secs(0),
        ))
        .await?;

        Ok(())
    }

    ///Stop scanning. Needed as currently the timeout for ScanSession does nothing.
    ///
    /// **Note**: Has no impact on extended scanning.
    /// Disabling scanning when it is already disabled has no effect.
    ///
    /// For more info:
    /// https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host-controller-interface/host-controller-interface-functional-specification.html#UUID-10327f75-4024-80df-14bc-68fe1e42b9e0:~:text=7.8.11.%20LE%20Set%20Scan%20Enable%20command
    pub async fn stop_scan(&mut self) -> Result<(), BleHostError<C::Error>>
    where
        C: ControllerCmdSync<LeSetScanEnable>,
    {
        let host = &self.central.host;
        let drop = crate::host::OnDrop::new(|| {
            host.scan_command_state().cancel(false);
        });
        host.scan_command_state().request(true).await; // FIXME: Is `true` correct?
        host.command(LeSetScanEnable::new(false, false)).await?;
        drop.defuse();
        Ok(())
    }

    /// Performs a BLE scan, return a report for discovering peripherals.
    ///
    /// Scan is stopped when a report is received. Call this method repeatedly to continue scanning.
    pub async fn scan(&mut self, config: &ScanConfig<'_>) -> Result<ScanSession<'stack, false>, BleHostError<C::Error>>
    where
        C: ControllerCmdSync<LeSetScanParams>
            + ControllerCmdSync<LeSetScanEnable>
            + ControllerCmdSync<LeClearFilterAcceptList>
            + ControllerCmdSync<LeAddDeviceToFilterAcceptList>,
    {
        let host = self.central.host;
        let drop = crate::host::OnDrop::new(|| {
            host.scan_command_state().cancel(false);
        });
        host.request_operation(host.scan_command_state(), false).await;

        if !config.filter_accept_list.is_empty() {
            self.central.set_accept_filter(config.filter_accept_list).await?;
        }

        let params = LeSetScanParams::new(
            if config.active {
                bt_hci::param::LeScanKind::Active
            } else {
                bt_hci::param::LeScanKind::Passive
            },
            bt_hci_duration(config.interval),
            bt_hci_duration(config.window),
            host.own_addr_kind(),
            if config.filter_accept_list.is_empty() {
                bt_hci::param::ScanningFilterPolicy::BasicUnfiltered
            } else {
                bt_hci::param::ScanningFilterPolicy::BasicFiltered
            },
        );
        host.command(params).await?;

        let filter_duplicates = !matches!(config.filter_duplicates, FilterDuplicates::Disabled);
        host.command(LeSetScanEnable::new(true, filter_duplicates)).await?;
        drop.defuse();
        Ok(ScanSession {
            command_state: self.central.host.scan_command_state(),
            deadline: if config.timeout.as_ticks() == 0 {
                None
            } else {
                Some(Instant::now() + config.timeout)
            },
            timeout: host.scan_timeout(),
            done: false,
        })
    }
}

/// Handle to an active advertiser which can accept connections.
pub struct ScanSession<'d, const EXTENDED: bool> {
    command_state: &'d CommandState<bool>,
    deadline: Option<Instant>,
    timeout: &'d Signal<NoopRawMutex, ()>,
    done: bool,
}

impl<const EXTENDED: bool> ScanSession<'_, EXTENDED> {
    /// Stop scanning and wait until the controller has disabled scanning.
    pub async fn stop(mut self) {
        self.command_state.cancel(EXTENDED);
        self.command_state.wait_idle().await;
        self.done = true;
    }
}

impl<const EXTENDED: bool> Drop for ScanSession<'_, EXTENDED> {
    fn drop(&mut self) {
        if !self.done {
            self.command_state.cancel(EXTENDED);
        }
    }
}

impl<'d> Future for ScanSession<'d, true> {
    type Output = ();

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> core::task::Poll<Self::Output> {
        let this = self.get_mut();
        core::pin::pin!(this.timeout.wait()).poll(cx)
    }
}
