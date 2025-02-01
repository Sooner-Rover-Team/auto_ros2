use futures_lite::StreamExt;
use serde::{Deserialize, Serialize};
use std::{
    net::{IpAddr, Ipv4Addr},
    time::Duration,
};

use ros2_client::{
    log::LogLevel, ros2::QosPolicyBuilder, rosout, AService, Context, Message, Name, Node,
    NodeName, NodeOptions, ServiceMapping, ServiceTypeName,
};

use feedback::{prelude::RoverController, Led};

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
    pub flashing: bool, //TODO: add implementation for flashing
}
impl Message for LightsRequest {}

// Struct to hold the response information (a boolean for success)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LightsResponse {
    pub success: bool,
}
impl Message for LightsResponse {}

#[tokio::main(flavor = "multi_thread")]
async fn main() {
    log4rs::init_file("log4rs.yaml", Default::default()).expect("logging initialization");

    // init ros2 context, which is just the DDS middleware
    let ros2_context = Context::new().expect("init ros 2 context");

    // create the lights node
    let mut node = create_node(&ros2_context);
    let service_qos = qos();

    rosout!(node, LogLevel::Info, "lights node is online!");

    // create server
    let server = node
        .create_server::<AService<LightsRequest, LightsResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/", "lights_service").unwrap(),
            &ServiceTypeName::new("example_interfaces", "Lights"),
            service_qos.clone(),
            service_qos,
        )
        .expect("create server");

    rosout!(
        node,
        LogLevel::Info,
        "Server created, waiting for requests..."
    );

    let ipaddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)); //TODO: Change this to the correct ip address
    let port = 8080; //TODO: Change this to the correct port number
                     // New instance of RoverController type, this should probably be a global thing. Need ip address and port number
    let controller = RoverController::new(ipaddr, port)
        .await
        .expect("Failed to create Rover Controller");
    // Create a task
    let mut server_stream = server.receive_request_stream();
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
                let lights_result = controller.send_led(&led).await;
                let response;
                match lights_result {
                    Ok(_) => {
                        response = LightsResponse { success: true };
                    }
                    Err(e) => {
                        rosout!(node, LogLevel::Error, "failed to send response: {e}");
                        response = LightsResponse { success: false };
                    }
                }

                let sr = server.async_send_response(req_id, response).await;

                if let Err(e) = sr {
                    rosout!(node, LogLevel::Error, "failed to send response: {e}");
                }
            }

            Err(e) => {
                rosout!(node, LogLevel::Error, "failed to send response: {e}");
            }
        }
    }

    // Make the node do stuff
    spin(&mut node);
    tokio::time::sleep(Duration::from_secs(30)).await;

    rosout!(node, LogLevel::Info, "lights node is shutting down...");
    //tracing::info!("lights node is shutting down...");
}

/// Creates the `lights_node`.
fn create_node(ctx: &Context) -> Node {
    // Create the lights node
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
