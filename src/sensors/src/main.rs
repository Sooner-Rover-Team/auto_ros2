//! # Sensors Node
//!
//! This node grabs data from each of the important sensors, providing it to
//! other nodes through various topics.

use core::net::{IpAddr, Ipv4Addr};
use std::{sync::Arc, time::Duration};

use camino::Utf8PathBuf;
use safe_drive::{
    clock::Clock,
    context::Context,
    logger::Logger,
    msg::{
        builtin_interfaces::UnsafeTime,
        common_interfaces::{
            geometry_msgs::msg::Quaternion,
            sensor_msgs::{
                self,
                msg::{Imu, COVARIANCE_TYPE_KNOWN, COVARIANCE_TYPE_UNKNOWN},
            },
            std_msgs::msg::Header,
        },
        RosString,
    },
    pr_error, pr_info, qos,
    topic::publisher::Publisher,
};

use feedback::parse::Message;
use safe_drive::msg::common_interfaces::{
    geometry_msgs::msg::Vector3, sensor_msgs::msg::NavSatFix,
};
use soro_gps::{Gps, GpsInfo};
use tokio::net::UdpSocket;

mod zed_imu;

const SENSORS_NODE_NAME: &str = "sensors_node";

#[tokio::main(flavor = "multi_thread")]
#[tracing::instrument]
async fn main() {
    // start logging
    tracing_subscriber::fmt()
        .with_env_filter(
            "RUST_LOG=ERROR,sensors_node=DEBUG,feedback=DEBUG,ros2_client[rosout_raw]=DEBUG,soro_gps=DEBUG,soro_sbp_gps=DEBUG",
        )
        .init();

    // create a context
    let ctx = Context::new().expect("init ros 2 context");

    // create the sensors node
    let sensors_node = ctx
        .create_node(SENSORS_NODE_NAME, Some("/sensors"), Default::default())
        .inspect_err(|e| tracing::error!("Failed to create `{SENSORS_NODE_NAME}`! err: {e}"))
        .expect("node creation");

    // create a logger
    let logger: Arc<Logger> = Arc::new(Logger::new(SENSORS_NODE_NAME));
    pr_info!(logger, "The `{SENSORS_NODE_NAME}` is now online!");

    // grab the sensor setup info
    let sensor_setup = SensorSetup::default();

    // make a gps (navsat) publisher
    let gps_publisher: Publisher<NavSatFix> = sensors_node
        .create_publisher("/sensors/gps", Some(qos::Profile::sensor_data()))
        .expect("create gps topic");

    // connect to gps
    //
    // note: we use `0` for the port we bind on since it doesn't matter.
    // we're just sending stuff to people and assuming they get it.
    let gps: Gps = Gps::new(sensor_setup.gps_path.into())
        .expect("gps creation (connection over serial) should be successful");

    // create the imu publisher
    let imu_publisher: Publisher<Imu> = sensors_node
        .create_publisher("/sensors/imu", Some(qos::Profile::sensor_data()))
        .expect("create imu topic");

    tokio::select![
        _ = tokio::signal::ctrl_c() => {},
        _ = tokio::task::spawn(gps_task(gps, gps_publisher)) => (),
        _ = tokio::task::spawn(_imu_task(Arc::clone(&logger), imu_publisher)) => (),
    ];
}

/// Information about how to reach the sensors.
///
/// For example, we'll try to reach the GPS at `{gps_ip}:{gps_port}`.
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SensorSetup {
    // gps
    pub gps_path: Utf8PathBuf,

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
        let gps_path: Utf8PathBuf = std::env::var("SORO_GPS_PATH")
            .map(Utf8PathBuf::from)
            .unwrap_or_else(|_| {
                "/dev/serial/by-id/usb-Septentrio_Septentrio_USB_Device_3844945-if02".into()
            });

        Self {
            gps_path,

            depth_camera_path: Utf8PathBuf::new(), // TODO: this is pretty bad
            mono_camera_path: Utf8PathBuf::new(),  // and this too

            battery_monitor_ip: ebox_microcontroller_ip,
            battery_monitor_port: 5007,

            imu_ip: ebox_microcontroller_ip,
            imu_port: 5006,
        }
    }
}

/// Publishes `GpsMessage`s when the GPS provides an update.
pub async fn gps_task(mut gps: Gps, gps_pub: Publisher<NavSatFix>) {
    // every 1/20th of a second, check for any updates.
    //
    // if the data is different, we'll publish it in a message.
    tracing::debug!("GPS task is up! Waiting for GPS data.");
    tracing::debug!("If a freeze occurs here, we're waiting to get any GPS data...");

    loop {
        // check for gps data
        let gps_data: GpsInfo = match gps.get() {
            Some(info) => info,
            Option::None => {
                tracing::warn!("Failed to get GPS data over serial!");
                tokio::time::sleep(Duration::from_secs(1) / 20).await;
                continue;
            }
        };

        // set up a header with the correct time and `frame_id`
        //
        // time first...
        let tow_ms = gps_data.tow.0;
        let stamp = UnsafeTime {
            sec: (tow_ms / 1000) as i32,

            // Time-of-week is in milliseconds.
            nanosec: (tow_ms % 1000) * 1_000_000,
        };
        //
        // ...and now the header itself
        const FRAME_ID: &str = "gps_link";
        let header = Header {
            stamp,
            frame_id: RosString::new(FRAME_ID).unwrap(),
        };

        // set locational info on a new `NavSatFix` message
        let mut nav_sat_fix = NavSatFix::new().expect("this won't fail lol");
        nav_sat_fix.header = header;
        nav_sat_fix.latitude = gps_data.coord.lat;
        nav_sat_fix.longitude = gps_data.coord.lon;
        nav_sat_fix.altitude = gps_data.height.0;

        // set the cov fields depending on what came back.
        //
        // TODO: use `GpsInfo.fix_status`
        nav_sat_fix.position_covariance = gps_data.covariance;
        if gps_data.covariance == [0_f64; 9] {
            nav_sat_fix.position_covariance_type = COVARIANCE_TYPE_UNKNOWN;
        } else {
            nav_sat_fix.position_covariance_type = COVARIANCE_TYPE_KNOWN;
        }
        tracing::debug!("Publishing GPS message: {nav_sat_fix:?}");

        // if we got all the values, publish them!
        _ = gps_pub
            .send(&nav_sat_fix)
            .inspect_err(|e| tracing::warn!("failed to write GPS message! err: {e}"))
            .inspect(|()| tracing::trace!("Published GPS message successfully!"));

        // sleep until gps can update
        //
        // note: this is, according to the GPS-RTK2's datasheet, "up to" 1/20th
        // of a second. the gps may not be able to update that fast, but even so,
        // we won't get a response in time if that's the case.
        //
        // so 1/20th of a second should work fine. :D
        tokio::time::sleep(Duration::from_secs(1) / 20).await;
    }
}

pub async fn _imu_task(logger: Arc<Logger>, imu_pub: Publisher<Imu>) {
    // connect to the IMU over this socket
    let sock: UdpSocket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, SensorSetup::default().imu_port))
        .await
        .expect("imu sock should connect");
    let mut buf: Vec<u8> = Vec::with_capacity(128);

    // grab the ROS 2 clock
    let mut clock: Clock = Clock::new().expect("clock is available when `rcl` is working right!");

    // grab data and parse
    loop {
        buf.clear();
        let recv_res = sock.recv_from(&mut buf).await;

        // make sure it was read alright
        let Ok((_bytes_read, _from_addr)) = recv_res else {
            tracing::warn!("failed to read");
            continue;
        };

        // parse into a struct
        let Ok(parsed_msg) = feedback::parse::parse(&buf) else {
            tracing::warn!("failed to parse");
            continue;
        };

        // make sure it's the right kind
        let Message::Imu(imu_raw) = parsed_msg else {
            tracing::warn!("wrong msg ty");
            continue;
        };

        // grab time before constructing message
        let time_ns = clock
            .get_now()
            .inspect_err(|e| {
                pr_error!(
                    logger,
                    "Failed to access time! `rcl` is unavailable. err: {e}"
                )
            })
            .expect("`rcl` should be available");

        // make a ROS 2 message from all that nonsense
        let imu_message = sensor_msgs::msg::Imu {
            header: Header {
                stamp: safe_drive::msg::builtin_interfaces__msg__Time {
                    sec: (time_ns / 1_000_000_000) as i32,
                    nanosec: (time_ns % 1_000_000_000) as u32,
                },
                frame_id: RosString::new("imu_link").expect("ros string creation"),
            },
            orientation: Quaternion {
                x: 1.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            },
            orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            angular_velocity: Vector3 {
                x: imu_raw.gyro_x as f64,
                y: imu_raw.gyro_y as f64,
                z: imu_raw.gyro_z as f64,
            },
            angular_velocity_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            linear_acceleration: Vector3 {
                x: imu_raw.accel_x as f64,
                y: imu_raw.accel_y as f64,
                z: imu_raw.accel_z as f64,
            },
            linear_acceleration_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };

        // publish it
        _ = imu_pub
            .send(&imu_message)
            .inspect_err(|e| tracing::error!("failed to publish imu msg. err: {e}"));
    }
}
