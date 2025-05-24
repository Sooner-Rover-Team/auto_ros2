//! Logic is encapsulated here to avoid clutter in the `main` module.

use std::{sync::Arc, time::Duration};

use safe_drive::{
    logger::Logger,
    msg::common_interfaces::sensor_msgs::msg::{Imu, NavSatFix},
    node::Node,
    pr_debug,
    qos::{
        policy::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy},
        Profile,
    },
    topic::publisher::Publisher,
};
use soro_gps::Gps;

use crate::SensorSetup;

// ros2-client to safe_drive NOTICE
// My attempt at recreating the DEFAULT_SUBSCRIPTION_QOS from ros2-client. Might be good to change...
//
/// Creates the set of QOS policies used to power the networking functionality.
///
/// Note that these are a little arbitrary. It might be nice to define them in
/// a shared crate library or at least find the 'best' values on the Rover
/// for competition.
#[tracing::instrument]
pub fn qos() -> Profile {
    Profile {
        history: HistoryPolicy::KeepLast,
        depth: 1,
        reliability: ReliabilityPolicy::BestEffort,
        durability: DurabilityPolicy::Volatile,
        deadline: Duration::MAX,
        lifespan: Duration::MAX,
        ..Default::default()
    }
}

/// Starts all tasks that over all sensor data and to publish to the correct topics.
pub async fn spawn_sensor_publisher_tasks(
    sensor_setup: impl AsRef<SensorSetup>,
    sensors_node: Arc<Node>,
    logger: Arc<Logger>,
) {
    let sensor_setup = sensor_setup.as_ref();
    // TODO: create mono cam, depth cam, battery, imu publishers

    // IMPORTANT: we lock the `sensors_node` for our own uses while we create
    // stuff for each task.
    //
    // when we're done, we'll unlock it, allowing them to start doing stuff.

    // spawn gps task
    {
        let gps_pubisher: Publisher<NavSatFix> = sensors_node
            .create_publisher("gps", Some(qos()))
            .expect("create gps topic");

        // connect to gps
        //
        // note: we use `0` for the port we bind on since it doesn't matter.
        // we're just sending stuff to people and assuming they get it.
        let gps = Gps::new(sensor_setup.gps_ip, sensor_setup.gps_port, 54555_u16)
            .await
            .expect("gps creation");

        tokio::task::spawn(sensor_tasks::gps_task(gps, gps_pubisher));
        pr_debug!(logger, "Made GPS task!");
    }

    // spawn imu task
    {
        // imu_task
        let imu_publisher: Publisher<Imu> = sensors_node
            .create_publisher("imu", Some(qos()))
            .expect("create imu topic");

        tokio::task::spawn(sensor_tasks::imu_task(imu_publisher));
        pr_debug!(logger, "Made IMU task");
    }
}

/// A module made of tasks for each sensor.
mod sensor_tasks {
    use std::{net::Ipv4Addr, time::Duration};

    use feedback::parse::Message;
    use safe_drive::{
        msg::common_interfaces::{
            geometry_msgs::msg::Vector3,
            sensor_msgs::msg::{Imu, NavSatFix},
        },
        topic::publisher::Publisher,
    };
    use soro_gps::Gps;
    use tokio::net::UdpSocket;

    use crate::SensorSetup;

    /// Publishes `GpsMessage`s when the GPS provides an update.
    pub async fn gps_task(mut gps: Gps, gps_pub: Publisher<NavSatFix>) {
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

            // // make a ros 2 msg from that info
            // let gps_message = GpsMessage {
            //     lat: gps_data.coord.lat,
            //     lon: gps_data.coord.lon,
            //     height: gps_data.height.0,
            //     error_mm: 0.0,
            //     time_of_week: gps_data.tow.0,
            // };

            // DOUBLE CHECK
            let mut nav_sat_fix = NavSatFix::new().expect("init NavSatFix message");
            nav_sat_fix.latitude = gps_data.coord.lat;
            nav_sat_fix.longitude = gps_data.coord.lon;
            nav_sat_fix.altitude = gps_data.height.0;
            // include error_mm?
            // include tow?

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
            sleep_gps().await;
        }

        async fn sleep_gps() {
            tokio::time::sleep(Duration::from_secs(1) / 20).await;
        }
    }

    pub async fn imu_task(imu_pub: Publisher<Imu>) {
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

            // // make a ros 2 message from that parsed info
            // let msg = ImuMessage {
            //     accel: Vector3 {
            //         x: imu_raw.accel_x,
            //         y: imu_raw.accel_y,
            //         z: imu_raw.accel_z,
            //     },
            //     gyro: Vector3 {
            //         x: imu_raw.gyro_x,
            //         y: imu_raw.gyro_y,
            //         z: imu_raw.gyro_z,
            //     },
            //     compass: Vector3 {
            //         x: imu_raw.compass_x,
            //         y: imu_raw.compass_y,
            //         z: imu_raw.compass_z,
            //     },
            //     temp_c: imu_raw.temp_c,
            // };

            // DOUBLE CHECK
            let mut imu_msg = Imu::new().expect("init Imu message");
            imu_msg.linear_acceleration = Vector3 {
                x: imu_raw.accel_x,
                y: imu_raw.accel_y,
                z: imu_raw.accel_z,
            };

            // Convert imu_raw.gyro_* Vector3 to Quaternion?
            // img_msg.orientation = Quaternion {
            //     x: todo!(),
            //     y: todo!(),
            //     z: todo!(),
            //     w: todo!(),
            // };

            // Complete assignment of fields

            // publish it
            _ = imu_pub
                .send(&imu_msg)
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
