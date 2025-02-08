use ros2_client::{
    log::LogLevel, prelude::NodeLoggingHandle, ros2::QosPolicyBuilder, rosout, Context, Node,
    NodeName, NodeOptions, Subscription,
};

use tokio_stream::StreamExt as _;

use crate::msg::WheelsMessage;
use feedback::{send::RoverController, Wheels};

/// Keeps watch for messages from other nodes asking for a change in wheel
/// speeds.
pub async fn wheels_task(
    subscriber: Subscription<WheelsMessage>,
    controller: RoverController,
    node_handle: NodeLoggingHandle,
) {
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
}

/// Creates the `wheels_node`.
pub fn create_node(ctx: &Context) -> Node {
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
pub fn spin(node: &mut Node) {
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
