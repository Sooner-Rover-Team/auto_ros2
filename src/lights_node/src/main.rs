//! # `lights_node`
//!
//! The lights node provides a service so that the client can make a request to turn the lights a given color.
//! The request information contains values representing:
//!
//! - RED
//! - GREEN
//! - BLUE
//!
//! and a boolean for FLASHING.

use serde::{Deserialize, Serialize};
use std::{
    net::{IpAddr, Ipv4Addr},
    time::Duration,
};

use ros2_client::{
    log::LogLevel, rosout, AService, Context, Message, Name, ServiceMapping, ServiceTypeName,
};

use feedback::prelude::RoverController;

mod logic;

// Struct to hold the request information (values for red, green, blue, and boolean for flashing)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LightsRequest {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
    pub flashing: bool, // TODO: add implementation for flashing
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
    // Initialize ros2 context, which is just the DDS middleware
    let ros2_context = Context::new().expect("init ros 2 context");

    // Create the lights node
    let mut node = logic::create_node(&ros2_context);
    let service_qos = logic::qos();

    rosout!(node, LogLevel::Info, "lights node is online!");

    // Create server
    let server = node
        .create_server::<AService<LightsRequest, LightsResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/", "lights_service").unwrap(),
            &ServiceTypeName::new("lights_node", "lights"),
            service_qos.clone(),
            service_qos,
        )
        .expect("create server");

    rosout!(
        node,
        LogLevel::Info,
        "Server created, waiting for requests..."
    );

    // Info for the controller to use
    let ebox_ipaddr = IpAddr::V4(Ipv4Addr::new(192, 168, 1, 68));
    let ebox_port = 5003;
    let local_port = 6666; // we bind to this port

    // New instance of RoverController type, this should probably be a global thing. Need ip address and port number
    let controller = RoverController::new(ebox_ipaddr, ebox_port, local_port)
        .await
        .expect("Failed to create Rover Controller");

    // Make the node do stuff
    logic::spin(&mut node);

    // Handle requests in the background
    tokio::task::spawn(logic::request_handler(server, node, controller));

    loop {
        tokio::time::sleep(Duration::from_secs(30)).await;
    }
}
