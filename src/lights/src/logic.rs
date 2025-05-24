use futures_lite::StreamExt;

use ros2_client::{
    log::LogLevel, ros2::QosPolicyBuilder, rosout, AService, Context, Node, NodeName, NodeOptions,
    Server,
};

use feedback::{prelude::RoverController, Led};

use crate::{LightsRequest, LightsResponse};

/// Handles service requests from other nodes.
///
/// This is what makes us the service "server".
pub async fn request_handler(
    server: Server<AService<LightsRequest, LightsResponse>>,
    node: Node,
    rover_controller: RoverController,
) {
    // make a stream out of the server's requests
    let mut server_stream = server.receive_request_stream();

    // loop over requests in the stream while we have some.
    //
    // if we don't have any, the task idles.
    while let Some(result) = server_stream.next().await {
        tracing::debug!("Entered while loop");
        match result {
            Ok((req_id, req)) => {
                rosout!(
                    node,
                    LogLevel::Info,
                    "received request: {:?} {:?}",
                    req_id,
                    req
                );

                // Send the request to the rover controller
                let led = Led {
                    red: req.red,
                    green: req.green,
                    blue: req.blue,
                };

                // This is what is sent back to the client (returns true if successful, false if not)
                let mut response = LightsResponse { success: false };

                // Try sending the led to the microcontroller
                if rover_controller
                    .send_led(&led)
                    .await
                    .inspect_err(|e| rosout!(node, LogLevel::Error, "failed to send request: {e}"))
                    .is_ok()
                {
                    rosout!(
                        node,
                        LogLevel::Debug,
                        "Lights value sent to microcontroller successfully!"
                    );
                    response = LightsResponse { success: true };
                }

                // Sends response to client
                let _ = server
                    .async_send_response(req_id, response)
                    .await
                    .inspect_err(|e| {
                        rosout!(node, LogLevel::Error, "failed to send response: {e}")
                    });
            }

            Err(e) => {
                rosout!(node, LogLevel::Error, "failed to send response: {e}");
            }
        }
    }
}

/// Creates the `lights_node`.
pub fn create_node(ctx: &Context) -> Node {
    // Create the lights node
    ctx.new_node(
        NodeName::new("/", "lights_node").expect("node naming"),
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
#[tracing::instrument(skip(node))]
pub fn spin(node: &mut Node) {
    tracing::debug!("Spinning node: `{}`", node.fully_qualified_name());

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
