use std::net::{IpAddr, Ipv4Addr};

use feedback::prelude::RoverController;
use logic::wheels_task;
use msg::WheelsMessage;
use ros2_client::{log::LogLevel, rosout, Context, MessageTypeName, Name, Subscription};

mod logic;
mod msg;

#[tokio::main(flavor = "multi_thread")]
async fn main() {
    // enable logging to terminal
    tracing_subscriber::fmt()
        .pretty()
        .with_max_level(tracing::Level::INFO)
        .init();

    // init ros2 context, which is just the DDS middleware
    let ros2_context = Context::new().expect("init ros 2 context");

    // create the wheels node
    let mut node = logic::create_node(&ros2_context);
    rosout!(node, LogLevel::Info, "wheels node is online!");

    // make the wheels topic
    let wheels_topic = node
        .create_topic(
            &Name::new("/rover", "wheels").unwrap(),
            MessageTypeName::new("std_msgs", "String"),
            &ros2_client::DEFAULT_SUBSCRIPTION_QOS,
        )
        .expect("create wheels topic");

    // message channel to handle incoming data

    // subscribe to wheels topic and forward messages to the channel
    //
    // FIXME: use dedicated message type
    let subscriber: Subscription<WheelsMessage> = node
        .create_subscription(
            &wheels_topic,
            Some(logic::qos()), // Apply the Quality of Service settings.
        )
        .expect("create subscription");

    // microcontroller that is responsible for controlling wheels
    // needs the microcontrollers' ip address and port
    let (ip, port): (Ipv4Addr, u16) = (Ipv4Addr::new(192, 168, 1, 102), 1002);
    let controller = RoverController::new(IpAddr::V4(ip), port, 0)
        .await
        .expect("Failed to create RoverController");

    // processes the recieved commands and sends the commands to the microcontroller
    let node_handle = node.logging_handle();
    tokio::task::spawn(wheels_task(subscriber, controller, node_handle));

    // connect dds
    logic::spin(&mut node);
}
