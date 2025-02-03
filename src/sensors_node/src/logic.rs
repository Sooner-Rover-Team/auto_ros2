//! Logic is encapsulated here to avoid clutter in the `main` module.

use std::sync::Arc;

use ros2_client::{
    log::LogLevel, ros2::QosPolicyBuilder, rosout, Context, MessageTypeName, Name, Node, NodeName,
    NodeOptions,
};
use soro_gps::Gps;
use tokio::sync::RwLock;

use crate::{msg::sensors::GpsMessage, SensorSetup};

/// Creates the `sensors_node`.
#[tracing::instrument(skip(ctx))]
pub fn create_node(ctx: &Context) -> Node {
    ctx.new_node(
        NodeName::new("/rustdds", "sensors_node").unwrap(),
        NodeOptions::new().enable_rosout(true),
    )
    .expect("node creation")
}

/// Creates the set of QOS policies used to power the networking functionality.
///
/// Note that these are a little arbitrary. It might be nice to define them in
/// a shared crate library or at least find the 'best' values on the Rover
/// for competition.
#[tracing::instrument]
pub fn qos() -> ros2_client::ros2::QosPolicies {
    QosPolicyBuilder::new()
        .history(ros2_client::ros2::policy::History::KeepLast { depth: 10 })
        .reliability(ros2_client::ros2::policy::Reliability::Reliable {
            max_blocking_time: ros2_client::ros2::Duration::from_millis(100),
        })
        .durability(ros2_client::ros2::policy::Durability::TransientLocal)
        .build()
}

/// Starts 'spinning' the given `Node`.
///
/// This spawns a background task that runs until the Node is turned off.
/// Without the spinner running, the Node is functionally useless. The same
/// is true for Nodes in Python.
#[tracing::instrument(skip(node))]
pub fn spin(node: &mut Node) {
    // try making the spinner task
    let spinner = node
        .spinner()
        .inspect_err(|e| tracing::error!("failed to make spinner! see: {e}"))
        .unwrap();

    tokio::task::spawn(async move {
        // note: this will 'spin' until the Node is dropped (meaning the
        // program is shutting down)
        let _ = spinner
            .spin()
            .await
            .inspect_err(|e| tracing::warn!("failed to spin node! see: {e}"));
    });
}

/// Starts all tasks that over all sensor data and to publish to the correct topics.
pub async fn spawn_sensor_publisher_tasks(
    sensor_setup: impl AsRef<SensorSetup>,
    sensors_node: Arc<RwLock<Node>>,
) {
    let sensor_setup = sensor_setup.as_ref();
    // TODO: create mono cam, depth cam, battery, imu publishers

    // IMPORTANT: we lock the `sensors_node` for our own uses while we create
    // stuff for each task.
    //
    // when we're done, we'll unlock it, allowing them to start doing stuff.

    // spawn gps task
    {
        let mut locked_sensors_node = sensors_node.write().await;

        let gps_topic = locked_sensors_node
            .create_topic(
                &Name::new("/sensors", "gps").expect("valid topic name"),
                MessageTypeName::new("sensors_node", "GpsMessage"),
                &qos(),
            )
            .expect("create gps topic");

        // make a gps publisher
        let gps_pub = locked_sensors_node
            .create_publisher::<GpsMessage>(&gps_topic, Some(qos()))
            .expect("create gps publisher");

        // connect to gps
        //
        // note: we use `0` for the port we bind on since it doesn't matter.
        // we're just sending stuff to people and assuming they get it.
        let gps =
            Gps::new(sensor_setup.gps_ip, sensor_setup.gps_port, 0_u16).expect("gps creation");
    }
}

