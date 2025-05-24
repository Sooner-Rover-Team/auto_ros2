//! # Sensors Node
//!
//! This node grabs data from each of the important sensors, providing it to
//! other nodes through various topics.

use core::net::{IpAddr, Ipv4Addr};
use std::time::Duration;

use camino::Utf8PathBuf;
use ros2_client::{log::LogLevel, rosout, Context};

mod logic;
mod msg;

#[tokio::main(flavor = "multi_thread")]
#[tracing::instrument]
async fn main() {
    // start logging
    tracing_subscriber::fmt()
        .with_env_filter(
            "RUST_LOG=ERROR,sensors_node=DEBUG,feedback=DEBUG,ros2_client[rosout_raw]=DEBUG,soro_gps=DEBUG,soro_sbp_gps=DEBUG",
        )
        .init();

    let ctx = Context::new().expect("init ros 2 context");

    // create the autonomous node
    let mut sensors_node = logic::create_node(&ctx);
    rosout!(
        sensors_node,
        LogLevel::Info,
        "sensors node is now starting!"
    );

    // make the node do stuff
    logic::spin(&mut sensors_node);

    // start all our topics + publishers
    //
    // TODO: maybe fill `sensor_setup` using params?
    let logging_handle = sensors_node.logging_handle();
    logic::spawn_sensor_publisher_tasks(SensorSetup::default(), sensors_node).await;
    rosout!(
        logging_handle,
        LogLevel::Info,
        "spawned all sensor info publishers"
    );

    loop {
        tokio::time::sleep(Duration::from_millis(20_000)).await;
    }
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
            gps_port: 54555, // FIXME: this should be documented! check first though.

            depth_camera_path: Utf8PathBuf::new(), // TODO: this is pretty bad
            mono_camera_path: Utf8PathBuf::new(),  // and this too

            battery_monitor_ip: ebox_microcontroller_ip,
            battery_monitor_port: 5007,

            imu_ip: ebox_microcontroller_ip,
            imu_port: 5006,
        }
    }
}
