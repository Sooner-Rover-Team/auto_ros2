//! Logic is encapsulated here to avoid clutter in the `main` module.

use ros2_client::{
    log::LogLevel, rosout, Context, MessageTypeName, Name, Node, NodeName, NodeOptions,
};
use soro_gps::Gps;

use crate::{
    msg::sensors::{GpsMessage, ImuMessage},
    SensorSetup,
};

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
    ros2_client::DEFAULT_SUBSCRIPTION_QOS.clone()
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
    mut sensors_node: Node,
) {
    let sensor_setup = sensor_setup.as_ref();
    // TODO: create mono cam, depth cam, battery, imu publishers

    // IMPORTANT: we lock the `sensors_node` for our own uses while we create
    // stuff for each task.
    //
    // when we're done, we'll unlock it, allowing them to start doing stuff.

    // spawn gps task
    {
        let gps_topic = sensors_node
            .create_topic(
                &Name::new("/sensors", "gps").expect("valid topic name"),
                MessageTypeName::new("custom_interfaces", "GpsMessage"),
                &qos(),
            )
            .expect("create gps topic");

        // make a gps publisher
        let gps_pub = sensors_node
            .create_publisher::<GpsMessage>(&gps_topic, Some(qos()))
            .expect("create gps publisher");

        // connect to gps
        //
        // note: we use `0` for the port we bind on since it doesn't matter.
        // we're just sending stuff to people and assuming they get it.
        let gps = Gps::new(sensor_setup.gps_ip, sensor_setup.gps_port, 54555_u16)
            .await
            .expect("gps creation");

        tokio::task::spawn(sensor_tasks::gps_task(gps, gps_pub));
        rosout!(sensors_node, LogLevel::Debug, "made gps task!");
    }

    // spawn imu task
    {
        // imu_task
        let imu_topic = sensors_node
            .create_topic(
                &Name::new("/sensors", "imu").expect("valid topic name"),
                MessageTypeName::new("custom_interfaces", "ImuMessage"),
                &qos(),
            )
            .expect("create imu topic");

        let imu_pub = sensors_node
            .create_publisher::<ImuMessage>(&imu_topic, Some(qos()))
            .expect("create imu publisher");

        tokio::task::spawn(sensor_tasks::imu_task(imu_pub));
        rosout!(sensors_node, LogLevel::Debug, "made gps task!");
    }
}

/// A module made of tasks for each sensor.
mod sensor_tasks {
    use std::{net::Ipv4Addr, time::Duration};

    use feedback::parse::Message;
    use ros2_client::Publisher;
    use soro_gps::Gps;
    use tokio::net::UdpSocket;

    use crate::{
        msg::{
            builtins::Vector3,
            sensors::{GpsMessage, ImuMessage},
        },
        SensorSetup,
    };

    /// Publishes `GpsMessage`s when the GPS provides an update.
    pub async fn gps_task(mut gps: Gps, gps_pub: Publisher<GpsMessage>) {
        // every 1/20th of a second, check for any updates.
        //
        // if the data is different, we'll publish it in a message.
        tracing::debug!("Trying to get GPS data from sensors node. If a freeze occurs here, we didn't see any GPS data.");
        loop {
            // check for gps data
            tracing::debug!("Trying to get GPS data from sensors node. If a freeze occurs here, we didn't see any GPS data.");
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
                lat: gps_data.coord.lat,
                lon: gps_data.coord.lon,
                height: gps_data.height.0,
                error_mm: 0.0,
                time_of_week: gps_data.tow.0,
            };
            tracing::debug!("Publishing GPS message: {gps_message:?}");

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

    pub async fn imu_task(imu_pub: Publisher<ImuMessage>) {
        let sock: UdpSocket =
            UdpSocket::bind((Ipv4Addr::UNSPECIFIED, SensorSetup::default().imu_port))
                .await
                .expect("imu sock should connect");
        let mut buf: Vec<u8> = Vec::with_capacity(128);

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

            // make a ros 2 message from that parsed info
            let msg = ImuMessage {
                accel: Vector3 {
                    x: imu_raw.accel_x,
                    y: imu_raw.accel_y,
                    z: imu_raw.accel_z,
                },
                gyro: Vector3 {
                    x: imu_raw.gyro_x,
                    y: imu_raw.gyro_y,
                    z: imu_raw.gyro_z,
                },
                compass: Vector3 {
                    x: imu_raw.compass_x,
                    y: imu_raw.compass_y,
                    z: imu_raw.compass_z,
                },
                temp_c: imu_raw.temp_c,
            };

            // publish it
            _ = imu_pub
                .async_publish(msg)
                .await
                .inspect_err(|e| tracing::error!("failed to publish imu msg. err: {e}"));
        }
    }
}

/*
#[cfg(test)]
mod tests {
    use std::net::Ipv4Addr;

    use ros2_client::{Context, MessageTypeName, Name, NodeName, NodeOptions};
    use soro_gps::Gps;
    use tokio::time::{timeout, Duration};

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
                MessageTypeName::new("custom_interfaces", "GpsMessage"),
                &super::qos(),
            )
            .unwrap();
        let gps_pub = node.create_publisher(&topic, None).unwrap();
        let gps = timeout(
            Duration::from_secs(2),
            Gps::new(Ipv4Addr::LOCALHOST.into(), 55556, 0),
        )
        .await
        .expect("gps should connect within seconds")
        .unwrap();

        // we ignore the error since we don't care if anything connects.
        //
        // this just ensures that the thread doesn't panic! :D
        let future = super::sensor_tasks::gps_task(gps, gps_pub);
        let _will_time_out = timeout(Duration::from_secs(2), future).await;
    }
}
*/
