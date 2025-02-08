use std::net::{IpAddr, Ipv4Addr};

use feedback::{prelude::RoverController, Wheels};
use msg::WheelsMessage;
use ros2_client::{
    log::LogLevel, ros2::QosPolicyBuilder, rosout, Context, MessageTypeName, Name, Node, NodeName,
    NodeOptions, Subscription,
};

use tokio_stream::StreamExt as _;

mod msg;

#[tokio::main(flavor = "multi_thread")]
async fn main() {
    // enable logging to terminal
    // tracing_subscriber::fmt()
    //     .pretty()
    //     .with_max_level(tracing::Level::INFO)
    //     .init();
    log4rs::init_file("log4rs.yaml", Default::default()).expect("logging initialization");

    // init ros2 context, which is just the DDS middleware
    let ros2_context = Context::new().expect("init ros 2 context");

    // create the wheels node
    let mut node = create_node(&ros2_context);
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
            Some(qos()), // Apply the Quality of Service settings.
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
    let join_handle = tokio::task::spawn(async move {
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
    {
        spin(&mut node);
    }

    if let Err(e) = join_handle.await {
        eprintln!("Error in join_handle task: {:?}", e);
    }

    // What is thine purpose?
    //tokio::time::sleep(Duration::from_secs(17)).await;

    tracing::info!("wheels node is shutting down...");
}

/// Creates the `wheels_node`.
fn create_node(ctx: &Context) -> Node {
    // make the wheels node
    ctx.new_node(
        NodeName::new("/rustdds", "wheels_node").expect("node naming"),
        NodeOptions::new().enable_rosout(true),
    )
    .expect("node creation")
}

/// Creates the set of QOS policies used to power the networking functionality.
///
/// Note that these are a little arbitrary. It might be nice to define them in
/// a shared crate library or at least find the 'best' values on the Rover
/// for competition.
fn qos() -> ros2_client::ros2::QosPolicies {
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
fn spin(node: &mut Node) {
    {
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
}
