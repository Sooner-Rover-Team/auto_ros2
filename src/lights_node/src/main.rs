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
use tokio::{sync::mpsc::{channel, Sender}, time::sleep};

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
    pub flashing: bool,
}
impl Message for LightsRequest {}

impl LightsRequest {
    pub fn green () -> Self {
        Self {
            red: 0,
            green: 255,
            blue: 0,
            flashing: false,
        }
    }

    pub fn off() -> Self {
        Self {
            red: 0,
            green: 0,
            blue: 0,
            flashing: false,
        }
    }
}

// Struct to hold the response information (a boolean for success)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LightsResponse {
    pub success: bool,
}
impl Message for LightsResponse {}

// Alternates requests to turn lights on (green) and off repeatedly
async fn FlashingLights(mut sender: Sender<LightsRequest>) {
    loop {
        // Turns lights green
        if sender.send(LightsRequest::green()).await.is_err() {
            break;
        }
        sleep(Duration::from_millis(200)).await;

        // Turns lights off
        if sender.send(LightsRequest::off()).await.is_err() {
            break;
        }
        sleep(Duration::from_millis(200)).await;
    }
}

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
    let ebox_ipaddr = IpAddr::V4(Ipv4Addr::new(192, 168, 1, 102));
    let ebox_port = 5003;
    let local_port = 6666; // we bind to this port

    // New instance of RoverController type, this should probably be a global thing. Need ip address and port number
    let controller = RoverController::new(ebox_ipaddr, ebox_port, local_port)
        .await
        .expect("Failed to create Rover Controller");

    // Channel to control light state
    let (sender, mut receiver) = channel::<LightsRequest>(32);

    // Receives color updates and applies them to hardware
    tokio::spawn({
        let controller = controller.clone();
        async move {
            while let Some(request) = receiver.recv().await {
                controller.set_lights_rgb(request.red, request.green, request.blue).await.ok();
            }
        }
    });

    // Service handler
    tokio::spawn({
        let sender = sender.clone();
        async move {
            while let Ok((request, response_tx)) = server.recv().await {
                if request.flashing {
                    let sender_clone = sender.clone();
                    tokio::spawn(async move {
                        FlashingLights(sender_clone).await;
                    });
                } else {
                    let _ = sender.send(request).await;
                }
                
                let _ = response_tx.send(LightsResponse { success: true });
            }
        }
    });

    // Node does things
    logic::spin(&mut node);

    // Keep main thread alive
    loop {
        tokio::time::sleep(Duration::from_secs(30)).await;
    }
}

#[cfg(test)]
mod tests;
