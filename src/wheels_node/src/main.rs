//use std::time::{Duration, Instant};
use feedback::{Wheels, prelude::RoverController, parse::Message};
use tokio::sync::{mpsc, Mutex};
use std::net::{Ipv4Addr, IpAddr};
use std::sync::Arc;

use ros2_client::{
    log::LogLevel, ros2::QosPolicyBuilder, rosout, Context, MessageTypeName, Name, Node, NodeName,
    NodeOptions, Subscription
};
// use tracing::level_filters::LevelFilter;

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
    let node = Arc::new(Mutex::new(create_node(&ros2_context)));
    rosout!(node.lock().await, LogLevel::Info, "wheels node is online!");

    // make sthe wheels topic
    let wheels_topic = {
        let node_locked = node.lock().await;
        let topic = node_locked.create_topic(
            &Name::new("/rover", "wheels").unwrap(),
            MessageTypeName::new("std_msgs", "String"),
            &ros2_client::DEFAULT_SUBSCRIPTION_QOS,
        ).expect("create wheels topic");

        topic
    };

    // message channel to handle incoming data
    // tx forwards messages recieved by topic
    // rx processes messages on arrival
    let (tx, rx) = mpsc::unbounded_channel::<String>();
    let rx = Arc::new(tokio::sync::Mutex::new(rx));
    // subscribe to wheels topic and forward messages to the channel
    let _subscriber: Subscription<String> = {
        let mut node_locked = node.lock().await;
        node_locked.create_subscription(
            &wheels_topic, 
            Some(qos()) // Apply the Quality of Service settings.
        ).expect("create subscription")
    };

    // ensures messges are processed correctly before being forwarded
    // prevents ownership/borrow conflicts with rx
    // : tokio::sync::mpsc::UnboundedReceiver<String>
    let rx_receiver = Arc::clone(&rx);
    tokio::task::spawn(async move {
        while let Some(msg) = rx_receiver.lock().await.recv().await { // blocking call to receive messages
            let _ = tx.send(msg).expect("Failed to send message"); // forward message to the channel
        }
    });

    // microcontroller that is responsible for controlling wheels
    // needs the microcontrollers' ip address and port
    let (ip, port): (Ipv4Addr, u16) = (Ipv4Addr::new(192, 168, 1, 102), 1002);
    let controller = RoverController::new(IpAddr::V4(ip), port).await.expect("Failed to create RoverController");

    // processes the recieved commands and sends the commands to the microcontroller
    let rx_sendwheels = Arc::clone(&rx);
    let node_clone = Arc::clone(&node);
    let join_handle = tokio::task::spawn(async move {
        while let Some(msg) = rx_sendwheels.lock().await.recv().await {
            match parse_wheels_msg(&msg) { // parses the message into the correct data
                Some(wheels) => {
                    // if the message is valid, send the parsed wheel commands to the microcontroller.
                    if let Err(e) = controller.send_wheels(&wheels).await {
                        let node_locked = node_clone.lock().await;
                        rosout!(node_locked, LogLevel::Error, "Failed to send wheels command: {e}");
                    }
                }
                None => {
                    // log a warning if the message format is incorrect.
                    let node_locked = node_clone.lock().await;
                    rosout!(node_locked, LogLevel::Warn, "Received invalid wheels message: {msg}");
                }
            }
        }
    });

    // make the node do stuff
    {
    let mut node_locked = node.lock().await;
    spin(&mut node_locked);
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

// converts the incoming message into the correct byte array based on feedback
#[tracing::instrument]
fn parse_wheels_msg(msg: &str) -> Option<Wheels> {
	let bytes = msg.as_bytes();

	let parsed_msg_res = feedback::parse::parse(bytes)
		.inspect_err(|e| tracing::debug!("Couldn't parse message! err: {e}"))
		.ok();

	if let Some(parsed_msg) = parsed_msg_res {
		if let Message::Wheels(wheels) = parsed_msg {
			tracing::debug!("Got a wheels message! {wheels:#?}");
			return Some(wheels);
		}
	}

	None
}
