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

use std::{
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
    time::Duration,
};

use custom_interfaces::srv::{Lights, Lights_Request, Lights_Response};

use feedback::{prelude::RoverController, Led};

use safe_drive::{
    context::Context,
    logger::Logger,
    node::{Node, NodeOptions},
    pr_error, pr_fatal, pr_info, qos,
    service::{
        server::{Server, ServerSend},
        Header,
    },
};

const LIGHTS_NODE_NAME: &str = "lights_node";

const LIGHTS_SERVICE_NAME: &str = "lights_service";

#[tokio::main(flavor = "multi_thread")]
async fn main() {
    // Initialize ros 2 context
    let ctx: Arc<Context> = Context::new().expect("init ros 2 context");

    // Create the logger
    let logger: Arc<Logger> = Arc::new(Logger::new(LIGHTS_NODE_NAME));

    // Create the lights node
    let node: Arc<Node> = ctx
        .create_node(LIGHTS_NODE_NAME, None, NodeOptions::new())
        .inspect_err(|e| pr_error!(logger, "Failed to create node: {e}"))
        .inspect(|_| {
            pr_info!(logger, "The `{LIGHTS_NODE_NAME}` has been created.");
        })
        .expect("create node");

    // Make the service server
    let server: Server<Lights> = node
        .create_server(LIGHTS_SERVICE_NAME, Some(qos::Profile::services_default()))
        .expect("create server");
    pr_info!(logger, "Server created! Starting request watcher...");

    // Info for the controller to use
    let ebox_ipaddr = IpAddr::V4(Ipv4Addr::new(192, 168, 1, 102));
    let ebox_port = 5003;
    let local_port = 6666; // we bind to this port

    // New instance of RoverController type, this should probably be a global
    // thing. Need ip address and port number
    let controller = RoverController::new(ebox_ipaddr, ebox_port, local_port)
        .await
        .expect("Failed to create Rover Controller");

    // Handle requests in the background
    tokio::select! {
        _ = tokio::signal::ctrl_c() => (),
        _ = service_server_task(server, controller, Arc::clone(&logger)) => (),
    };
}

async fn service_server_task(
    mut server: Server<Lights>,
    controller: RoverController,
    logger: Arc<Logger>,
) {
    // wait for requests
    loop {
        let maybe_request: Result<(ServerSend<_>, Lights_Request, Header), _> = server.recv().await;

        // we got a request! let's unwrap what it contains...
        let (responder, request, _header) = match maybe_request {
            // if there's no error, just steal its contents
            Ok(tup) => tup,

            // but if we get an error, report it, then wait for a new message
            Err(e) => {
                pr_fatal!(logger, "Failed to get message from request! err: {e}");

                // FIXME: would prefer this is `continue`, but we don't get
                // back a handle to the `server` in the error... D:
                return;
            }
        };

        // make a new RGB value with the request's color
        let led = Led {
            red: request.red,
            green: request.green,
            blue: request.blue,
        };

        // try sending the LED color to the microcontroller
        let success = match controller.send_led(&led).await {
            Ok(_) => true,
            Err(e) => {
                pr_error!(logger, "Failed to send light color to Ebox! err: {e}");
                false
            }
        };

        // send a response back based on how that went...
        //
        // (returns true if successful, false if not)
        let response = Lights_Response { success };

        // we'll try to send back the response.
        //
        // this can fail if `rcl` loses connection, so we have to `match on
        // the result of this operation to "get the server back" - it was
        // "consumed" by the request.
        //
        // in other words, we can only get back a server if we still have a
        // valid `rcl` connection
        let mut responder = responder;
        server = loop {
            match responder.send(&response) {
                Ok(s) => break s,
                Err((retry, e)) => {
                    pr_fatal!(
                    logger,
                    "`rcl` connection error detected! Waiting 1.0s before trying again. err: {e}"
                );

                    // retry after a moment
                    tokio::time::sleep(Duration::from_secs(1)).await;
                    responder = retry;
                }
            }
        };
    }
}

// #[cfg(test)]
// mod tests;
