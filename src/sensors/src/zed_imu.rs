use std::{f64::consts::PI, sync::Arc};

use async_hid::{AsyncHidRead as _, Device, HidBackend};
use futures_lite::StreamExt as _;
use safe_drive::{
    logger::Logger,
    msg::{
        common_interfaces::{
            geometry_msgs::msg::{Quaternion, Vector3},
            sensor_msgs,
            std_msgs::msg::Header,
        },
        RosString,
    },
    pr_error,
    topic::publisher::Publisher,
};
use zerocopy::{FromBytes, KnownLayout};

pub async fn zed_imu_publisher_task(
    logger: Arc<Logger>,
    publisher: Publisher<sensor_msgs::msg::Imu>,
) {
    let (mut device_reader, _device_writer) = HidBackend::default()
        .enumerate()
        .await
        .unwrap()
        .find(|dev: &Device| dev.matches(0x2b03, 0xf881, 0, 0))
        .await
        .expect("Could not find device")
        .open()
        .await
        .unwrap();

    // grab messages until we can't anymore!
    let mut buf;
    loop {
        buf = [0_u8; 64]; // clear buf
        let n = device_reader.read_input_report(&mut buf).await.unwrap();
        if n < 64 {
            pr_error!(
                logger,
                "Couldn't grab info from ZED IMU. (not long enough. expected: 64, got: {n}"
            );
            continue;
        }

        // we have enough bytes! let's cast into the type below...
        let imu_data = match RawData::read_from_bytes(&buf) {
            Ok(imu_data) => imu_data,

            Err(e) => {
                pr_error!(logger, "Parsing failed for ZED IMU info. err: {e}");
                continue;
            }
        };

        // grab imu timestamp as ns
        let timestamp_ns = (imu_data.timestamp as f64 * TS_SCALE_NS) as u64;

        // convert that imu data into a ROS 2 `sensors_msgs::msg::Imu`
        let imu_message = sensor_msgs::msg::Imu {
            header: Header {
                stamp: safe_drive::msg::builtin_interfaces__msg__Time {
                    sec: (timestamp_ns / 1_000_000_000) as i32,
                    nanosec: (timestamp_ns % 1_000_000_000) as u32,
                },
                frame_id: RosString::new("imu_link").expect("ros string creation"),
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            },
            orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            angular_velocity: Vector3 {
                x: (imu_data.g_x as f64 * GYRO_SCALE as f64) * (PI / 180.0),
                y: (imu_data.g_y as f64 * GYRO_SCALE as f64) * (PI / 180.0),
                z: (imu_data.g_z as f64 * GYRO_SCALE as f64) * (PI / 180.0),
            },
            angular_velocity_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            linear_acceleration: Vector3 {
                x: imu_data.g_x as f64 * ACC_SCALE as f64,
                y: imu_data.g_y as f64 * ACC_SCALE as f64,
                z: imu_data.g_z as f64 * ACC_SCALE as f64,
            },
            linear_acceleration_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };

        // send the message!
        _ = publisher.send(&imu_message).inspect_err(|e| {
            pr_error!(logger, "Failed to send IMU message! err: {e}");
        });
    }
}

/// Raw IMU data.
///
/// WARNING: PACKED STRUCT - HANDLE WITH CARE.
///
/// Layout defined by the Zed OSS team here:
/// https://github.com/stereolabs/zed-open-capture/blob/5cf66ff777175776451b9b59ecc6231d730fa202/include/sensorcapture_def.hpp#L78-L106
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, zerocopy::FromBytes, KnownLayout)]
pub struct RawData {
    pub struct_id: u8,
    pub imu_not_valid: u8,
    pub timestamp: u64,
    pub g_x: i16,
    pub g_y: i16,
    pub g_z: i16,
    pub a_x: i16,
    pub a_y: i16,
    pub a_z: i16,
    pub frame_sync: u8,
    pub sync_capabilities: u8,
    pub frame_sync_count: u32,
    pub imu_temp: i16,
    pub mag_valid: u8,
    pub m_x: i16,
    pub m_y: i16,
    pub m_z: i16,
    pub camera_moving: u8,
    pub camera_moving_count: u32,
    pub camera_falling: u8,
    pub camera_falling_count: u32,
    pub env_valid: u8,
    pub temp: i16,
    pub press: u32,
    pub humid: u32,
    pub temp_cam_left: i16,
    pub temp_cam_right: i16,

    // IMPORTANT: REQUIRED PADDING TO AVOID POTENTIAL UNDEFINED BEHAVIOR
    _padding: [u8; 2],
}

// constants taken from ZED OSS
const ACC_SCALE: f32 = 9.8189 * (8.0 / 32768.0);
const GYRO_SCALE: f32 = 1000.0 / 32768.0;
const _MAG_SCALE: f32 = 1.0 / 16.0;
const TS_SCALE_NS: f64 = 39_062.5;
