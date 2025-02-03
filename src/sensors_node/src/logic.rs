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

        tokio::task::spawn(sensor_tasks::gps_task(
            gps,
            gps_pub,
            Arc::clone(&sensors_node),
        ));
        rosout!(locked_sensors_node, LogLevel::Debug, "made gps task!");
    }
}

/// A module made of tasks for each sensor.
mod sensor_tasks {
    use std::{sync::Arc, time::Duration};

    use ros2_client::{log::LogLevel, rosout, Node, Publisher};
    use soro_gps::{Coordinate, ErrorInMm, Gps, Height, TimeOfWeek};
    use tokio::sync::RwLock;

    use crate::msg::{builtins::GeoPoint, sensors::GpsMessage};

    /// Publishes `GpsMessage`s when the GPS provides an update.
    pub async fn gps_task(gps: Gps, gps_pub: Publisher<GpsMessage>, node: Arc<RwLock<Node>>) {
        let mut coord_raw: Option<Coordinate>;
        let mut height_raw: Option<Height>;
        let mut error_raw: Option<ErrorInMm>;
        let mut tow_raw: Option<TimeOfWeek>;

        // every 1/20th of a second, check for any updates.
        //
        // if the data is different, we'll publish it in a message.
        loop {
            // check for gps data
            coord_raw = gps.coord();
            height_raw = gps.height();
            error_raw = gps.error();
            tow_raw = gps.time_of_week();

            // if we got all the values, publish them!
            if let (Some(coord), Some(height), Some(error), Some(tow)) =
                (coord_raw, height_raw, error_raw, tow_raw)
            {
                let gps_message = GpsMessage {
                    coord: GeoPoint {
                        latitude: coord.lat,
                        longitude: coord.lon,
                        altitude: height.0,
                    },
                    error_mm: error.0,
                    time_of_week: tow.0,
                };
                tracing::debug!("Attempting to publish GPS message: {gps_message:?}");

                _ = gps_pub
                    .async_publish(gps_message)
                    .await
                    .inspect_err(|e| {
                        rosout!(
                            node.blocking_read(), // FIXME: blocking might be bad here
                            LogLevel::Warn,
                            "failed to write GPS message! err: {e}"
                        )
                    })
                    .inspect(|()| tracing::trace!("Published GPS message successfully!"));
            }

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
}

