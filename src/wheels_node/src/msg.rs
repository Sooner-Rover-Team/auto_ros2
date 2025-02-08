/// A message with info about the wheels.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct WheelsMessage {
    pub left_wheels: u8,
    pub right_wheels: u8,
}
impl ros2_client::Message for WheelsMessage {}
