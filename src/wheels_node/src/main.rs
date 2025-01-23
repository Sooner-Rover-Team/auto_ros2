use std::time::{Duration, Instant};
use feedback::{Wheels, prelude::RoverController};

use ros2_client::{
    log::LogLevel, ros2::QosPolicyBuilder, rosout, Context, MessageTypeName, Name, Node, NodeName,
    NodeOptions,
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
    let mut node = create_node(&ros2_context);
    rosout!(node, LogLevel::Info, "wheels node is online!");

    // make some topic to 'chat' on
    let chatter_topic = node
        .create_topic(
            &Name::new("/", "topic").unwrap(),
            MessageTypeName::new("std_msgs", "String"),
            &ros2_client::DEFAULT_SUBSCRIPTION_QOS,
        )
        .expect("make chatter topic");

    // and make a publisher for it
    let publisher = node
        .create_publisher::<String>(&chatter_topic, Some(qos()))
        .expect("create publisher");

    // make the node do stuff
    spin(&mut node);

    // finally, publish for ~15s
    tokio::task::spawn(async move {
        let start_time = Instant::now();

        let mut msg_ct: u8 = 0;

        // run for 15s after the task starts running
        while start_time.elapsed() < std::time::Duration::from_secs(15) {
            msg_ct += 1;
            let _ = publisher
                .async_publish(format!(
                    "hey navigator... we started {} ms ago",
                    start_time.elapsed().as_millis()
                ))
                .await
                .inspect_err(|e| tracing::warn!("failed to publish! see: {e}"));

            rosout!(node, LogLevel::Warn, "sent a message! msg_ct: {msg_ct}");

            // sleep for 500ms each loop
            tokio::time::sleep(Duration::from_millis(500)).await;
        }

        // after this prints, the future is Poll::Ready(()), so we're done with
        // our task. this will stop running and won't run again.
        tracing::info!("done publishing! see ya");
    });

    tokio::time::sleep(Duration::from_secs(17)).await;

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
