//! BLE on the micro:bit: the remote's side of the link to the robot. The remote is the
//! *central*: it scans for an advertisement that lists the robot's service UUID
//! (`balance_core::link::SERVICE_UUID`), connects to whoever sent it, finds the packet
//! characteristic and writes a packet into it. For now that is a fixed one (inactive,
//! speed and turn 0, `seq` counting up) once a second; live tilt commands come next. When
//! the connection drops it starts scanning again. [`connected`] tells the display task
//! whether it is connected.
//!
//! Three layers, bottom up, as on the robot:
//! - **Controller**: Nordic's `SoftDevice` Controller (SDC), a closed-source link layer
//!   linked in as a static library, on top of the Multiprotocol Service Layer (MPSL),
//!   which owns the radio's timing: the RADIO, TIMER0 and RTC0 interrupts at the highest
//!   priority (0), plus a low-priority one (EGU0/SWI0) for the work that can wait.
//!   `nrf-sdc` / `nrf-mpsl` wrap them.
//! - **HCI**: here a function call; `SoftdeviceController` implements `bt-hci`'s
//!   `Controller` directly.
//! - **Host**: `trouble-host`, the same host as on the robot, here in the central role
//!   and with a GATT *client* (the robot runs the server).
//!
//! Finding the characteristic: GATT data is a table of attributes, each with a handle (its
//! row number, assigned by the server). The client asks the robot for the service with
//! our UUID (the range of handles it spans), then within it for the characteristic with
//! the packet UUID (its value handle). Writes go to that handle.

use core::sync::atomic::{AtomicBool, Ordering};

use balance_core::{
    link::{self, PACKET_UUID, Packet, SERVICE_UUID},
    remote::Command,
};
use embassy_executor::Spawner;
use embassy_futures::{
    join::join,
    select::{Either, select},
};
use embassy_nrf::{Peri, peripherals, rng};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker, Timer};
use nrf_mpsl::{MultiprotocolServiceLayer, raw as mpsl_raw};
use nrf_sdc::{self as sdc, SoftdeviceController};
use static_cell::StaticCell;
use trouble_host::prelude::*;

use crate::Irqs;

/// The micro:bit's BLE address: a fixed "random static" one (top two bits of the last
/// byte set), like the robot's.
const ADDRESS: [u8; 6] = [0x17, 0x5e, 0xb1, 0x7a, 0x2c, 0xd3];

/// One robot at a time.
const CONNECTIONS: usize = 1;
/// L2CAP channels: the ones a connection always has (ATT for GATT, the signalling one).
const L2CAP_CHANNELS: usize = 2;
/// Services the GATT client remembers: only ours.
const MAX_SERVICES: usize = 1;

/// How often the fixed packet is written.
const PACKET_PERIOD: Duration = Duration::from_secs(1);
/// Wait before scanning again after something failed.
const RETRY_DELAY: Duration = Duration::from_secs(1);

/// The connection counts as lost when nothing is heard from the robot for this long
/// (both sides then give up on it). The default is 8 s, which would make the remote
/// notice a robot reset only after 8 s; 2 s still rides out a short radio dropout.
const SUPERVISION_TIMEOUT: Duration = Duration::from_secs(2);

/// Memory for the `SoftDevice` Controller (connection and buffer state), in bytes.
/// `build` checks it and logs the size it needs ("Memory buffer too big/small") if this
/// doesn't match; 2136 for one central link with the default buffers.
const SDC_MEM_SIZE: usize = 2136;

/// The SDC, through `trouble-host`'s eyes.
type Sdc = SoftdeviceController<'static>;
/// The host error type with the SDC underneath.
type HostError = BleHostError<sdc::Error>;

/// Whether a robot is connected; read by the display task.
static CONNECTED: AtomicBool = AtomicBool::new(false);

/// The address of an advertiser that lists our service, from the scan callback to the
/// task that connects. A `Signal` keeps only the latest value.
static FOUND: Signal<CriticalSectionRawMutex, Address> = Signal::new();

static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();
static RNG: StaticCell<rng::Rng<'static, embassy_nrf::mode::Async>> = StaticCell::new();
static SDC_MEM: StaticCell<sdc::Mem<SDC_MEM_SIZE>> = StaticCell::new();

/// Whether the remote is connected to a robot.
pub fn connected() -> bool {
    CONNECTED.load(Ordering::Relaxed)
}

/// The peripherals MPSL and the SDC take over; the application must not touch them
/// (nor CCM, AAR and NVMC). On the nRF52: RTC0, TIMER0, TEMP and PPI channels 17-31.
pub struct Hardware {
    /// For MPSL.
    pub mpsl: nrf_mpsl::Peripherals<'static>,
    /// For the SDC.
    pub sdc: sdc::Peripherals<'static>,
    /// The hardware random number generator: the SDC needs random numbers (e.g. for
    /// encryption keys and channel selection).
    pub rng: Peri<'static, peripherals::RNG>,
}

/// Starts MPSL and the SDC and spawns the tasks that run them and the link. A failure
/// is logged and leaves the remote without BLE (the tilt display still works).
pub fn start(spawner: Spawner, hw: Hardware) {
    // The low-frequency clock (32.768 kHz, keeps time between radio events): the internal
    // RC oscillator, recalibrated against the crystal every 4 s (`rc_ctiv` in 0.25 s
    // units), and every 8 s at the latest when the temperature is steady
    // (`rc_temp_ctiv` intervals). That keeps it within 500 ppm, which MPSL needs to know
    // to widen its receive windows. Nordic's recommended values.
    #[allow(
        clippy::cast_possible_truncation,
        reason = "the C header's constants are u32, but all are small (≤ 500)"
    )]
    let lfclk = mpsl_raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl_raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl_raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl_raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl_raw::MPSL_WORST_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: false,
    };
    let mpsl = match MultiprotocolServiceLayer::new(hw.mpsl, Irqs, lfclk) {
        Ok(mpsl) => MPSL.init(mpsl),
        Err(e) => {
            defmt::error!("ble: MPSL init failed: {}", e);
            return;
        }
    };
    let rng = RNG.init(rng::Rng::new(hw.rng, Irqs));
    let sdc = sdc::Builder::new()
        .and_then(|b| b.support_scan().support_central().central_count(1))
        .and_then(|b| b.build(hw.sdc, rng, mpsl, SDC_MEM.init(sdc::Mem::new())));
    let sdc = match sdc {
        Ok(sdc) => sdc,
        Err(e) => {
            defmt::error!("ble: `SoftDevice` Controller init failed: {}", e);
            return;
        }
    };
    match mpsl_task(mpsl) {
        Ok(task) => spawner.spawn(task),
        Err(e) => defmt::error!("ble: MPSL task not started: {}", e),
    }
    match central(sdc) {
        Ok(task) => spawner.spawn(task),
        Err(e) => defmt::error!("ble: central task not started: {}", e),
    }
}

/// Runs MPSL's low-priority work (woken by its EGU0/SWI0 interrupt), forever.
#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

/// Receives the advertising reports the controller passes up while scanning.
struct ScanHandler;

impl EventHandler for ScanHandler {
    fn on_adv_reports(&self, reports: LeAdvReportsIter) {
        for report in reports.flatten() {
            if link::advertises_service(report.data, SERVICE_UUID) {
                FOUND.signal(Address {
                    kind: report.addr_kind,
                    addr: report.addr,
                });
            }
        }
    }
}

/// Runs the BLE host and the link, forever. As on the robot, the host's `runner` must
/// run all the time (here it also hands advertising reports to `ScanHandler`), next to
/// the code that uses the host.
#[embassy_executor::task]
async fn central(sdc: Sdc) {
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS, L2CAP_CHANNELS> =
        HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources)
        .set_random_address(Address::random(ADDRESS))
        .build();
    let mut runner = stack.runner();
    let mut central = stack.central();
    let (result, ()) = join(
        runner.run_with_handler(&ScanHandler),
        link_loop(&stack, &mut central),
    )
    .await;
    if let Err(e) = result {
        defmt::error!("ble: host stopped: {}", e);
    }
}

/// Scan, connect, write packets until the connection drops; again, forever.
async fn link_loop(
    stack: &Stack<'_, Sdc, DefaultPacketPool>,
    central: &mut Central<'_, Sdc, DefaultPacketPool>,
) {
    loop {
        defmt::info!("ble: scanning for the robot");
        let robot = match find_robot(central).await {
            Ok(robot) => robot,
            Err(e) => {
                defmt::warn!("ble: scan failed: {}", e);
                Timer::after(RETRY_DELAY).await;
                continue;
            }
        };
        defmt::info!("ble: found the robot at {}, connecting", robot);

        let config = ConnectConfig {
            scan_config: ScanConfig {
                filter_accept_list: &[robot],
                ..ScanConfig::default()
            },
            connect_params: RequestedConnParams {
                supervision_timeout: SUPERVISION_TIMEOUT,
                ..RequestedConnParams::default()
            },
        };
        let connection = match central.connect(&config).await {
            Ok(connection) => connection,
            Err(e) => {
                defmt::warn!("ble: connect failed: {}", e);
                Timer::after(RETRY_DELAY).await;
                continue;
            }
        };
        defmt::info!("ble: connected");
        CONNECTED.store(true, Ordering::Relaxed);
        let result = send_packets(stack, &connection).await;
        CONNECTED.store(false, Ordering::Relaxed);
        match result {
            Err(BleHostError::BleHost(Error::Disconnected)) => {
                defmt::info!("ble: disconnected");
            }
            Err(e) => defmt::warn!("ble: link ended: {}", e),
            Ok(()) => {}
        }
    }
}

/// Scans (passively: the service UUID is in the advertisement itself, so no scan
/// request is needed) until `ScanHandler` hears the robot, then stops the scan and waits
/// until the controller has really stopped: it refuses to start a connection while it
/// still scans ("Command Disallowed"). Dropping the session would only ask it to stop.
async fn find_robot(
    central: &mut Central<'_, Sdc, DefaultPacketPool>,
) -> Result<Address, HostError> {
    FOUND.reset();
    let mut scanner = Scanner::new(central);
    let config = ScanConfig {
        active: false,
        ..ScanConfig::default()
    };
    let session = scanner.scan(&config).await?;
    let robot = FOUND.wait().await;
    session.stop().await;
    Ok(robot)
}

/// Finds the packet characteristic and writes the fixed packet every `PACKET_PERIOD`
/// until the connection drops. The GATT client's `task` receives the robot's replies
/// (to the discovery requests) and must run next to the requests.
async fn send_packets(
    stack: &Stack<'_, Sdc, DefaultPacketPool>,
    connection: &Connection<'_, DefaultPacketPool>,
) -> Result<(), HostError> {
    let client = GattClient::<Sdc, DefaultPacketPool, MAX_SERVICES>::new(stack, connection).await?;
    let writes = async {
        let services = client
            .services_by_uuid(&Uuid::new_long(link::uuid_bytes(SERVICE_UUID)))
            .await?;
        let service = services.first().ok_or(Error::NotFound)?;
        let packet: Characteristic<[u8; link::LEN]> = client
            .characteristic_by_uuid(service, &Uuid::new_long(link::uuid_bytes(PACKET_UUID)))
            .await?;
        defmt::info!("ble: found the packet characteristic, writing once a second");

        let mut seq: u8 = 0;
        let mut ticker = Ticker::every(PACKET_PERIOD);
        loop {
            let bytes = Packet::new(seq, Command::default(), false).encode();
            // Without response: a BLE *command*, no acknowledgement per packet, so the
            // next one never waits for a round trip (what 50 packets/s will need).
            client
                .write_characteristic_without_response(&packet, &bytes)
                .await?;
            defmt::info!("ble: sent {=[u8]:02x}", bytes);
            seq = seq.wrapping_add(1);
            ticker.next().await;
        }
    };
    match select(client.task(), writes).await {
        Either::First(result) | Either::Second(result) => result,
    }
}
