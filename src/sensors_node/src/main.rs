//! # Sensors Node
//!
//! This node grabs data from each of the important sensors, providing it to
//! other nodes through various topics.

use core::net::{IpAddr, Ipv4Addr};

use camino::Utf8PathBuf;
use ros2_client::{log::LogLevel, rosout, Context};

mod logic;
mod msg;

#[tokio::main(flavor = "multi_thread")]
#[tracing::instrument]
async fn main() {
    // start logging
    tracing_subscriber::fmt()
        .pretty()
        .with_env_filter("RUST_LOG=debug")
        .init();

    let ctx = Context::new().expect("init ros 2 context");

    // create the autonomous node
    let mut sensors_node = logic::create_node(&ctx);
    rosout!(sensors_node, LogLevel::Info, "sensors node is online!");

    // make the node do stuff
    logic::spin(&mut sensors_node);
}

/// Information about how to reach the sensors.
///
/// For example, we'll try to reach the GPS at `{gps_ip}:{gps_port}`.
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SensorSetup {
    // gps
    pub gps_ip: IpAddr,
    pub gps_port: u16,

    // mono + depth cameras
    pub depth_camera_path: Utf8PathBuf,
    pub mono_camera_path: Utf8PathBuf,

    // battery
    pub battery_monitor_ip: IpAddr,
    pub battery_monitor_port: u16,

    // imu
    pub imu_ip: IpAddr,
    pub imu_port: u16,
}

impl AsRef<SensorSetup> for SensorSetup {
    fn as_ref(&self) -> &SensorSetup {
        self
    }
}

impl Default for SensorSetup {
    fn default() -> Self {
        let ebox_microcontroller_ip = Ipv4Addr::new(192, 168, 1, 102).into();

        Self {
            gps_ip: IpAddr::V4(Ipv4Addr::new(192, 168, 1, 222)),
            gps_port: 55556, // FIXME: this should be documented! check first though.

            depth_camera_path: Utf8PathBuf::new(), // TODO: this is pretty bad
            mono_camera_path: Utf8PathBuf::new(),  // and this too

            battery_monitor_ip: ebox_microcontroller_ip, // FIXME: assuming it's just the ebox microcontroller
            battery_monitor_port: 5007,

            imu_ip: ebox_microcontroller_ip, // FIXME: assuming it's just the ebox microcontroller
            imu_port: 5006,
        }
    }
}
