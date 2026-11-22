//! BLE on the robot. So far only advertising: the robot shows up as `BalanceRobot` in a
//! scanner (bring-up step E6). It doesn't accept connections yet; the GATT service the
//! remote writes to comes next.
//!
//! Three layers, bottom up:
//! - **Controller**: Espressif's BLE controller, a closed-source blob in the radio
//!   firmware that does the timing-critical link layer (radio, channels, packets).
//!   `esp-radio` runs it on `esp-rtos`'s scheduler and wraps it as a `BleConnector`.
//! - **HCI** (Host Controller Interface): the standard byte protocol between a BLE
//!   controller and its host, normally a UART between two chips; here a function call.
//!   `bt-hci`'s `ExternalController` speaks it over the connector.
//! - **Host**: `trouble-host`, the protocol stack above it (GAP: advertising and
//!   connections; later GATT: the data the remote writes).

use bt_hci::controller::ExternalController;
use core::fmt::Write as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use esp_hal::peripherals::BT;
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

use crate::{LOG, LogMsg};

/// The name a scanner shows.
const NAME: &[u8] = b"BalanceRobot";

/// The robot's BLE address: a fixed "random static" address (the two top bits of the
/// last byte set, as the spec requires for that kind), so a phone sees the same device
/// after every reset. Fine for one robot; the remote will find it by its service UUID.
const ADDRESS: [u8; 6] = [0x42, 0xa1, 0x7e, 0x05, 0xb0, 0xc5];

/// HCI commands that can wait for their reply at the same time.
const HCI_SLOTS: usize = 8;
/// Connections and L2CAP channels the host keeps room for: one remote at a time.
const CONNECTIONS: usize = 1;
const L2CAP_CHANNELS: usize = 1;

/// Brings the BLE controller up and spawns the task that advertises. Called during setup,
/// before the log task owns the USB serial, so failures here use `println!`.
pub(crate) fn start(spawner: Spawner, bt: BT<'static>) {
    let connector = match BleConnector::new(bt, esp_radio::ble::Config::default()) {
        Ok(connector) => connector,
        Err(e) => {
            println!("# ble: controller init failed: {e:?}");
            return;
        }
    };
    match advertise(connector) {
        Ok(task) => spawner.spawn(task),
        Err(e) => println!("# ble task not started: {e:?}"),
    }
}

/// Logs one `# ble: ...` line through the log task. A line that doesn't fit is cut off.
async fn log(args: core::fmt::Arguments<'_>) {
    let mut text = heapless::String::new();
    let _ = text.write_fmt(args);
    LOG.send(LogMsg::Ble(text)).await;
}

/// Runs the BLE host and advertises, forever.
///
/// The host's `runner` processes everything the controller sends (events, data) and must
/// run all the time; `join` runs it next to the code that uses the host. Both live in
/// this one task, so the host's resources can simply be locals here: the task's future
/// is stored in a `static`, so they live as long as needed.
#[embassy_executor::task]
async fn advertise(connector: BleConnector<'static>) {
    let controller: ExternalController<_, HCI_SLOTS> = ExternalController::new(connector);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS, L2CAP_CHANNELS> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random(ADDRESS))
        .build();
    let mut runner = stack.runner();
    let mut peripheral = stack.peripheral();

    // Advertising data: at most 31 bytes of "AD structures" (length, type, data).
    // Flags say "discoverable, BLE only"; the name is what the scanner lists.
    let mut adv_data = [0; 31];
    let len = match AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(NAME),
        ],
        &mut adv_data,
    ) {
        Ok(len) => len,
        Err(e) => {
            log(format_args!("advertising data too long: {e:?}")).await;
            return;
        }
    };

    let advertising = async {
        // Non-connectable for now: nothing would answer a connection yet. Default
        // parameters: every 160 ms on all three advertising channels, 0 dBm.
        let params = AdvertisementParameters::default();
        let data = Advertisement::NonconnectableScannableUndirected {
            adv_data: &adv_data[..len],
            scan_data: &[],
        };
        match peripheral.advertise(&params, data).await {
            // Advertising runs as long as the `Advertiser` lives, so keep it.
            Ok(_advertiser) => {
                log(format_args!("advertising as BalanceRobot")).await;
                core::future::pending::<()>().await;
            }
            Err(e) => log(format_args!("advertising failed: {e:?}")).await,
        }
    };
    let (result, ()) = join(runner.run(), advertising).await;
    if let Err(e) = result {
        log(format_args!("host stopped: {e:?}")).await;
    }
}
