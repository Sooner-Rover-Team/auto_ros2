//! # Sensors Node
//!
//! This node grabs data from each of the important sensors, providing it to
//! other nodes through various topics.

use ros2_client::{
    log::LogLevel, ros2::QosPolicyBuilder, rosout, Context, Node, NodeName, NodeOptions,
};

#[tokio::main(flavor = "multi_thread")]
#[tracing::instrument]
async fn main() {
    // start logging
    tracing_subscriber::fmt()
        .pretty()
        .with_env_filter("RUST_LOG=debug")
        .init();

    let ctx = Context::new().expect("init ros 2 context");

    // create the autonomous node
    let mut sensors_node = create_node(&ctx);
    rosout!(sensors_node, LogLevel::Info, "sensors node is online!");

    // make the node do stuff
    spin(&mut sensors_node);
}

/// Creates the `sensors_node`.
#[tracing::instrument(skip(ctx))]
fn create_node(ctx: &Context) -> Node {
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
fn _qos() -> ros2_client::ros2::QosPolicies {
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
fn spin(node: &mut Node) {
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
