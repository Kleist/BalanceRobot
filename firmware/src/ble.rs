//! BLE on the robot: the link to the micro:bit remote. The robot advertises as
//! `BalanceRobot` with its GATT service, accepts one connection at a time, and logs every
//! packet written to the service's packet characteristic as `# remote: ...`, decoded with
//! `balance_core::link`. A `Watchdog` notices when packets stop. Nothing is applied to the
//! motors yet.
//!
//! Three layers, bottom up:
//! - **Controller**: Espressif's BLE controller, a closed-source blob in the radio
//!   firmware that does the timing-critical link layer (radio, channels, packets).
//!   `esp-radio` runs it on `esp-rtos`'s scheduler and wraps it as a `BleConnector`.
//! - **HCI** (Host Controller Interface): the standard byte protocol between a BLE
//!   controller and its host, normally a UART between two chips; here a function call.
//!   `bt-hci`'s `ExternalController` speaks it over the connector.
//! - **Host**: `trouble-host`, the protocol stack above it. GAP: advertising and
//!   connections. GATT: a table of *attributes* (services containing *characteristics*,
//!   each a small value with a UUID and a handle) that the connected device reads and
//!   writes. The remote writes each packet into one characteristic.

use balance_core::link::{self, PACKET_UUID, Packet, SERVICE_UUID, Watchdog};
use core::fmt::Write as _;
use embassy_executor::Spawner;
use embassy_futures::{
    join::join,
    select::{Either, select},
};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::peripherals::BT;
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

use crate::{LOG, LogMsg};

/// The name a scanner shows (and the GAP service's device name).
const NAME: &str = "BalanceRobot";

/// The robot's BLE address: a fixed "random static" address (the two top bits of the
/// last byte set, as the spec requires for that kind), so a phone sees the same device
/// after every reset. Fine for one robot; the remote will find it by its service UUID.
const ADDRESS: [u8; 6] = [0x42, 0xa1, 0x7e, 0x05, 0xb0, 0xc5];

/// HCI commands that can wait for their reply at the same time.
const HCI_SLOTS: usize = 8;
/// One remote at a time.
const CONNECTIONS: usize = 1;
/// L2CAP channels: the ones a connection always has (ATT for GATT, the signalling one).
const L2CAP_CHANNELS: usize = 2;

/// How often the watchdog is checked while connected.
const WATCHDOG_PERIOD: Duration = Duration::from_millis(50);

/// The remote's service: one characteristic it writes packets into. Writable with and
/// without response: "without" (a BLE *command*) is what the remote will use, since it
/// needs no acknowledgement per packet; "with" (a *request*) is what most phone apps do
/// by default.
#[gatt_service(uuid = link::uuid_bytes(SERVICE_UUID))]
struct RemoteService {
    /// The last packet written (`balance_core::link`).
    #[characteristic(uuid = link::uuid_bytes(PACKET_UUID), write, write_without_response)]
    packet: [u8; link::LEN],
}

/// The robot's GATT server: the standard GAP service (device name) plus ours.
#[gatt_server]
struct Server {
    remote: RemoteService,
}

/// Brings the BLE controller up and spawns the task that runs the link. Called during
/// setup, before the log task owns the USB serial, so failures here use `println!`.
pub(crate) fn start(spawner: Spawner, bt: BT<'static>) {
    let connector = match BleConnector::new(bt, esp_radio::ble::Config::default()) {
        Ok(connector) => connector,
        Err(e) => {
            println!("# ble: controller init failed: {e:?}");
            return;
        }
    };
    match ble_link(connector) {
        Ok(task) => spawner.spawn(task),
        Err(e) => println!("# ble task not started: {e:?}"),
    }
}

/// Logs one `# ble: ...` / `# remote: ...` line (the text starts with the prefix)
/// through the log task. A line that doesn't fit is cut off.
async fn log(args: core::fmt::Arguments<'_>) {
    let mut text = heapless::String::new();
    let _ = text.write_fmt(args);
    LOG.send(LogMsg::Ble(text)).await;
}

/// Microseconds since boot, the watchdog's clock.
fn now_us() -> u64 {
    Instant::now().as_micros()
}

/// Runs the BLE host and the link, forever.
///
/// The host's `runner` processes everything the controller sends (events, data) and must
/// run all the time; `join` runs it next to the code that uses the host. Both live in
/// this one task, so the host's resources can simply be locals here: the task's future
/// is stored in a `static`, so they live as long as needed.
#[embassy_executor::task]
async fn ble_link(connector: BleConnector<'static>) {
    let controller: ExternalController<_, HCI_SLOTS> = ExternalController::new(connector);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS, L2CAP_CHANNELS> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random(ADDRESS))
        .build();
    let mut runner = stack.runner();
    let mut peripheral = stack.peripheral();

    let server = match Server::new_default(NAME) {
        Ok(server) => server,
        Err(e) => {
            log(format_args!("ble: GATT server setup failed: {e}")).await;
            return;
        }
    };

    let (result, ()) = join(runner.run(), serve(&mut peripheral, &server)).await;
    if let Err(e) = result {
        log(format_args!("ble: host stopped: {e:?}")).await;
    }
}

/// Advertises, serves one connection until it ends, and starts over.
async fn serve<C: Controller>(
    peripheral: &mut Peripheral<'_, C, DefaultPacketPool>,
    server: &Server<'_>,
) {
    // Advertising data: at most 31 bytes of "AD structures" (length, type, data). Flags
    // (3 bytes) and the 128-bit service UUID (18) fill most of it, so the name (14) goes
    // into the scan response, which a scanner asks for and gets another 31 bytes.
    let mut adv_data = [0; 31];
    let mut scan_data = [0; 31];
    let lens = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteServiceUuids128(&[link::uuid_bytes(SERVICE_UUID)]),
        ],
        &mut adv_data,
    )
    .and_then(|adv| {
        AdStructure::encode_slice(
            &[AdStructure::CompleteLocalName(NAME.as_bytes())],
            &mut scan_data,
        )
        .map(|scan| (adv, scan))
    });
    let (adv_len, scan_len) = match lens {
        Ok(lens) => lens,
        Err(e) => {
            log(format_args!("ble: advertising data too long: {e:?}")).await;
            return;
        }
    };

    loop {
        // Default parameters: every 160 ms on all three advertising channels, 0 dBm.
        let advertiser = match peripheral
            .advertise(
                &AdvertisementParameters::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &adv_data[..adv_len],
                    scan_data: &scan_data[..scan_len],
                },
            )
            .await
        {
            Ok(advertiser) => advertiser,
            Err(e) => {
                log(format_args!("ble: advertising failed: {e:?}")).await;
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };
        log(format_args!("ble: advertising as {NAME}")).await;
        // Advertising stops by itself when a device connects.
        let connection = match advertiser.accept().await {
            Ok(connection) => connection,
            Err(e) => {
                log(format_args!("ble: accept failed: {e:?}")).await;
                continue;
            }
        };
        let peer = connection.peer_address().addr;
        // Stored least significant byte first; shown the usual way round, as on the phone.
        if let [a0, a1, a2, a3, a4, a5] = peer.raw() {
            log(format_args!(
                "ble: connected to {a5:02X}:{a4:02X}:{a3:02X}:{a2:02X}:{a1:02X}:{a0:02X}"
            ))
            .await;
        }
        match connection.with_attribute_server(&server.server) {
            Ok(gatt) => handle_connection(&gatt, server).await,
            Err(e) => log(format_args!("ble: GATT setup failed: {e:?}")).await,
        }
    }
}

/// Handles one connection's events until it disconnects: logs every packet write and
/// checks the watchdog every `WATCHDOG_PERIOD`.
async fn handle_connection(gatt: &GattConnection<'_, '_, DefaultPacketPool>, server: &Server<'_>) {
    let mut watchdog = Watchdog::new(link::TIMEOUT_US);
    loop {
        match select(gatt.next(), Timer::after(WATCHDOG_PERIOD)).await {
            Either::First(GattConnectionEvent::Disconnected { reason }) => {
                watchdog.on_disconnect();
                log(format_args!(
                    "ble: disconnected ({reason:?}); remote: targets 0"
                ))
                .await;
                return;
            }
            Either::First(GattConnectionEvent::Gatt { event }) => {
                if let GattEvent::Write(write) = &event
                    && write.handle() == server.remote.packet.handle
                {
                    // The write's bytes are only lent to the closure, so decode them
                    // there and keep a copy of the first few for the log.
                    let (decoded, raw, raw_len) = write.with_data(|_offset, bytes| {
                        (Packet::decode(bytes), first_bytes(bytes), bytes.len())
                    });
                    match decoded {
                        Ok(packet) => {
                            watchdog.on_packet(now_us(), packet);
                            let command = packet.command();
                            log(format_args!(
                                "remote: seq {} active {} speed {:.2} turn {:.2}",
                                packet.seq,
                                u8::from(packet.active),
                                command.speed,
                                command.turn
                            ))
                            .await;
                        }
                        Err(e) => {
                            let shown = &raw[..raw_len.min(raw.len())];
                            log(format_args!(
                                "remote: rejected {shown:02x?} ({raw_len} bytes): {e:?}"
                            ))
                            .await;
                        }
                    }
                }
                // Let the server store the value and, for a write *request*, send the
                // acknowledgement (a *command* gets none).
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => log(format_args!("ble: GATT reply failed: {e:?}")).await,
                }
            }
            // Connection parameter and PHY updates and so on: nothing to do.
            Either::First(_) => {}
            Either::Second(()) => {
                if watchdog.poll(now_us()) {
                    log(format_args!("remote: link timeout, targets 0")).await;
                }
            }
        }
    }
}

/// The first 8 bytes of a write (fewer if it is shorter; the rest stays 0), for the log.
fn first_bytes(bytes: &[u8]) -> [u8; 8] {
    let mut first = [0; 8];
    let n = bytes.len().min(first.len());
    first[..n].copy_from_slice(&bytes[..n]);
    first
}
