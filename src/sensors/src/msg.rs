//! Message types for our sensors.

/// These are built into ROS 2 itself. They're here since `ros2-client` does
/// not provide them.
pub mod builtins {
    use ros2_client::Message;

    /// A message type in ROS 2: `msg:geographic_msgs/GeoPoint`.
    #[derive(Clone, Debug, PartialEq, PartialOrd, serde::Deserialize, serde::Serialize)]
    pub struct GeoPoint {
        pub latitude: f64,
        pub longitude: f64,
        pub altitude: f64,
    }
    impl Message for GeoPoint {}

    /// A message type in ROS 2: `msg:geometry_msgs/Vector3`.
    #[derive(Clone, Debug, PartialEq, PartialOrd, serde::Deserialize, serde::Serialize)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    impl Message for Vector3 {}
}

/// Sensor-specific message types.
pub mod sensors {
    use ros2_client::Message;

    use super::builtins::Vector3;

    /// A message made from GPS data.
    ///
    /// Sent to `topic:sensors/gps`.
    #[derive(Clone, Debug, PartialEq, PartialOrd, serde::Deserialize, serde::Serialize)]
    pub struct GpsMessage {
        pub lat: f64,
        pub lon: f64,
        pub height: f64,
        pub error_mm: f64,
        pub time_of_week: u32,
    }
    impl Message for GpsMessage {}

    /// A message made of the `ICM-20948` IMU's sensor data.
    ///
    /// Sent to `topic:sensors/imu`.
    #[derive(Clone, Debug, PartialEq, PartialOrd, serde::Deserialize, serde::Serialize)]
    pub struct ImuMessage {
        pub accel: Vector3,

        pub gyro: Vector3,

        /// `y` is the typical compass value, but we also measure the `x` and
        /// `z` values for tilt information.
        ///
        /// These values are in degrees, and will always be within the range of
        /// 0.0 to 360.0.
        pub compass: Vector3,

        pub temp_c: f64,
    }
    impl Message for ImuMessage {}

    /*









    FIXME
    the following messages are incomplete. you shouldn't expect them to be
    filled yet.
    */

    /// Depth camera data.
    ///
    /// Sent to `topic:sensors/depth_image`.
    //
    // TODO: i have no clue what these will look like.
    //
    // maybe rosbag?
    #[derive(Clone, Debug, PartialEq, PartialOrd, serde::Deserialize, serde::Serialize)]
    pub struct DepthCameraMessage {}
    impl Message for DepthCameraMessage {}

    /// Camera data.
    ///
    /// Sent to `topic:sensors/mono_image`.
    //
    // TODO: see above.
    #[derive(Clone, Debug, PartialEq, PartialOrd, serde::Deserialize, serde::Serialize)]
    pub struct MonoCameraMessage {}
    impl Message for MonoCameraMessage {}

    /// Battery charge information.
    ///
    /// Assists with managing batteries, potentially via the RoverGUI, in the
    /// future. Sent to `topic:sensors/battery`.
    //
    // TODO: we don't yet know what data the battery sensors will get.
    #[derive(Clone, Debug, PartialEq, PartialOrd, serde::Deserialize, serde::Serialize)]
    pub struct BatteryMessage {}
    impl Message for BatteryMessage {}
}
