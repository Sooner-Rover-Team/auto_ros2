use std::net::{IpAddr, Ipv4Addr};

use feedback::{prelude::RoverController, Wheels};
use msg::WheelsMessage;
use ros2_client::{log::LogLevel, rosout, Context, MessageTypeName, Name, Subscription};

use tokio_stream::StreamExt as _;

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
    tokio::task::spawn(async move {
        let mut stream = Box::pin(subscriber.async_stream());

        while let Some(msg_result) = stream.next().await {
            // get the data from it or log an err!
            let (msg, _msg_info) = match msg_result {
                Ok(t) => t,
                Err(e) => {
                    rosout!(
                        node_handle,
                        LogLevel::Warn,
                        "Failed to parse message data! err: {e}"
                    );
                    continue;
                }
            };

            // make message into something we can send to the microcontroller.
            let wheels = Wheels {
                wheel0: msg.left_wheels,
                wheel1: msg.left_wheels,
                wheel2: msg.left_wheels,
                wheel3: msg.right_wheels,
                wheel4: msg.right_wheels,
                wheel5: msg.right_wheels,
                checksum: 0, // FIXME: this should be in the `feedback` crate!
            };

            // try sending it
            if let Err(e) = controller.send_wheels(&wheels).await {
                rosout!(
                    node_handle,
                    LogLevel::Error,
                    "Failed to send wheels command: {e}"
                );
            }
        }
    });

    // make the node do stuff
    logic::spin(&mut node);
}
