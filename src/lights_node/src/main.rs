use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};

use ros2_client::{
    log::LogLevel, ros2::QosPolicyBuilder, rosout, Context, MessageTypeName, Name, Node, NodeName,
    NodeOptions, AService, ServiceMapping, ServiceTypeName,
};

use feedback::*;

/* The lights node provides a service so that the client can make a request to turn the lights a given color.
The request information contains values representing:
    RED
    GREEN
    BLUE
    and a boolean for FLASHING.
*/

// Struct to hold the request information (values for red, green, blue, and boolean for flashing)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LightsRequest {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
    pub flashing: bool,
}
impl Message for LightsRequest {}

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

    // create the lights node
    let mut node = create_node(&ros2_context);
    let service_qos = qos();
    rosout!(node, LogLevel::Info, "lights node is online!");

    // create server
    let server = node
        .create_server::<AService<LightsRequest, SendResult>>(
            ServiceMapping::Enhanced,
            &Name::new("/", "lights_service").unwrap(),
            &ServiceTypeName::new("lights", "Lights"),
            service_qos.clone(),
            service_qos
        )
        .expect("create server");

    rosout!(node, LogLevel::Info, "server created!");

    // make the node do stuff
    spin(&mut node);

    // New instance of RoverController type, this should probably be a global thing. Need ip address and port number
    let controller = RoverController:new(ipaddr, port).await.expect("Failed to create Rover Controller");
    // Create a task
    tokio::task::spawn(async move {
        let start_time = Instant::now();

        let server_stream = server.receive_request_stream().then(|result| async {
            match result {
                Ok((req_id, req)) => {
                    async_publish!(node, LogLevel::Info, "received request: {req_id} {req}");
                    // Parse incoming message into byte array based on feedback
                    // let response = controller.send_led(&leds)
                    let response = LightsResponse { success: true };
                    let sr = server.send_response(req_id, response).await;
                    if let Err(e) = sr {
                        async_publish!(node, LogLevel::Error, "failed to send response: {e}");
                    }
                }
                Err(e) => {
                    async_publish!(node, LogLevel::Error, "failed to receive request: {e}");
            }
        }
        });

        // run the server
        server_stream.await;

    });

    tokio::time::sleep(Duration::from_secs(17)).await;

    tracing::info!("lights node is shutting down...");
}

/// Creates the `lights_node`.
fn create_node(ctx: &Context) -> Node {
    // make the lights node
    ctx.new_node(
        NodeName::new("/rustdds", "lights_node").expect("node naming"),
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
