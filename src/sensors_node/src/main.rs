//! # Sensors Node
//!
//! This node grabs data from each of the important sensors, providing it to
//! other nodes through various topics.

use ros2_client::{log::LogLevel, rosout, Context};

mod logic;
mod msg;

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
    let mut sensors_node = logic::create_node(&ctx);
    rosout!(sensors_node, LogLevel::Info, "sensors node is online!");

    // make the node do stuff
    logic::spin(&mut sensors_node);
}
