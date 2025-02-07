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
        let gps = Gps::new(sensor_setup.gps_ip, sensor_setup.gps_port, 0_u16)
            .await
            .expect("gps creation");

        tokio::task::spawn(sensor_tasks::gps_task(gps, gps_pub));
        rosout!(locked_sensors_node, LogLevel::Debug, "made gps task!");
    }
}

/// A module made of tasks for each sensor.
mod sensor_tasks {
    use std::time::Duration;

    use ros2_client::Publisher;
    use soro_gps::Gps;

    use crate::msg::{builtins::GeoPoint, sensors::GpsMessage};

    /// Publishes `GpsMessage`s when the GPS provides an update.
    pub async fn gps_task(mut gps: Gps, gps_pub: Publisher<GpsMessage>) {
        // every 1/20th of a second, check for any updates.
        //
        // if the data is different, we'll publish it in a message.
        loop {
            // check for gps data
            let gps_data = match gps.get().await {
                Ok(info) => info,
                Err(e) => {
                    tracing::warn!("Failed to get GPS data from sensors node! err: {e}");
                    sleep_gps().await;
                    continue;
                }
            };

            // make a ros 2 msg from that info
            let gps_message = GpsMessage {
                coord: GeoPoint {
                    latitude: gps_data.coord.lat,
                    longitude: gps_data.coord.lon,
                    altitude: gps_data.height.0,
                },
                error_mm: 0.0,
                time_of_week: gps_data.tow.0,
            };
            tracing::debug!("Attempting to publish GPS message: {gps_message:?}");

            // if we got all the values, publish them!
            _ = gps_pub
                .async_publish(gps_message)
                .await
                .inspect_err(|e| tracing::warn!("failed to write GPS message! err: {e}"))
                .inspect(|()| tracing::trace!("Published GPS message successfully!"));

            // sleep until gps can update
            //
            // note: this is, according to the GPS-RTK2's datasheet, "up to" 1/20th
            // of a second. the gps may not be able to update that fast, but even so,
            // we won't get a response in time if that's the case.
            //
            // so 1/20th of a second should work fine. :D
            sleep_gps().await;
        }

        async fn sleep_gps() {
            tokio::time::sleep(Duration::from_secs(1) / 20).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use std::net::Ipv4Addr;

    use ros2_client::{Context, MessageTypeName, Name, NodeName, NodeOptions};
    use soro_gps::Gps;

    #[tokio::test]
    async fn gps_task_doesnt_panic() {
        let ctx = Context::new().unwrap();
        let mut node = ctx
            .new_node(
                NodeName::new("/test", "gps_task_doesnt_panic_node").unwrap(),
                NodeOptions::new(),
            )
            .unwrap();
        let topic = node
            .create_topic(
                &Name::new("/test", "gps_task_doesnt_panic_topic").unwrap(),
                MessageTypeName::new("sensors", "GpsMessage"),
                &super::qos(),
            )
            .unwrap();
        let gps_pub = node.create_publisher(&topic, None).unwrap();
        let gps = Gps::new(Ipv4Addr::LOCALHOST.into(), 55556, 0)
            .await
            .unwrap();

        // we ignore the error since we don't care if anything connects.
        //
        // this just ensures that the thread doesn't panic! :D
        let future = super::sensor_tasks::gps_task(gps, gps_pub);
        let _will_time_out =
            tokio::time::timeout(tokio::time::Duration::from_secs(2), future).await;
    }
}
