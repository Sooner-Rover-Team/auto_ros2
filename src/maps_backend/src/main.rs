use safe_drive::{
    context::Context, msg::common_interfaces::sensor_msgs, node::Node, qos::Profile,
    topic::subscriber::Subscriber,
};
use sensor_msgs::msg::NavSatFix;

use futures_util::{SinkExt, StreamExt};
use std::sync::Arc;
use tokio::{net::TcpListener, sync::broadcast};
use tokio_tungstenite::accept_async;
use tungstenite::{Message, Utf8Bytes};

const MAPS_NODE: &str = "maps_node";

#[tokio::main]
#[tracing::instrument]
async fn main() {
    // start `tracing` crate logging
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::DEBUG)
        .init();

    // Set up ROS2 context and node
    tracing::debug!("Creating ROS 2 node...");
    let ctx: Arc<Context> = Context::new().unwrap();
    let node: Arc<Node> = ctx
        .create_node(MAPS_NODE, Some("/"), Default::default())
        .unwrap();
    tracing::debug!("ROS 2 node created!");

    tracing::debug!("Creating GPS subscriber...");
    // Set up GPS subscriber
    let gps_subscriber = node
        .create_subscriber::<NavSatFix>("/sensors/gps", Some(Profile::sensor_data()))
        .unwrap();
    tracing::debug!("GPS subscriber created!");

    // Set up broadcast channel to share GPS data with WebSocket clients
    tracing::debug!("Making thread broadcast channel...");
    let (tx, rx) = broadcast::channel::<String>(16); // 16 is the buffer size
    tracing::debug!("Completed creating thread broadcast channel!");

    // now, we'll spawn three tasks...
    tokio::select! {
        // this one ensures that if you hit Ctrl^C, the program stops
        // immediately (or almost immediately)
        _ = tokio::signal::ctrl_c() => (),

        // this task will handle ros 2 messages (from the gps/navsat) in the
        // background.
        //
        // when it get messages, it sends it to our "websocket" task that's up
        // next
        _ = tokio::spawn(handle_ros2_messages(gps_subscriber, tx)) => (),

        // finally, this'll listen for messages from the ROS 2 subscriber task.
        //
        // when it gets anything, it'll immediately send the info to the
        // frontend
        _ = tokio::spawn(start_websocket_server(rx)) => (),
    }
}

/// Watches for ROS 2 `NavSatFix` messages.
///
/// When it gets one, it'll tell the other thread to send that information to
/// the frontend over a WebSocket connection.
#[tracing::instrument(skip(sub))]
async fn handle_ros2_messages(mut sub: Subscriber<NavSatFix>, tx: broadcast::Sender<String>) {
    tracing::info!("Now handling ROS 2 messsages!");

    loop {
        match sub.recv().await {
            Ok(msg) => {
                // make a small JSON message with the msg's lat + lon
                let lat: f64 = msg.latitude;
                let lon: f64 = msg.longitude;

                // ensure that neither is NaN
                if !lat.is_finite() || !lon.is_finite() {
                    tracing::error!(
                        "Either lat or lon is NaN/infinite! Not sending message. got: ({lat:?}, {lon:?})"
                    );
                    return;
                }

                let json: String = create_json_coordinate_pair(lat, lon);

                // debug print it
                tracing::debug!("Sending the following JSON: {json}");

                // if the other thread died, we'll stop too!
                match tx.send(json) {
                    Ok(bytes_sent) => tracing::debug!("Sent {bytes_sent} bytes to other thread!"),
                    Err(e) => {
                        tracing::error!("Other thread is dead! err: {e}");
                        return;
                    }
                }
            }

            Err(e) => {
                tracing::error!("Failed to receive GPS data! err: {e}");
            }
        }
    }
}

/// The correct format, in JSON, for the frontend, is as follows:
///
/// `{"lat": A.Bbbbbbb, "lon": C.Dddddd}`
fn create_json_coordinate_pair(lat: f64, lon: f64) -> String {
    format!(r#"{{"lat": {:?}, "lon": {:?}}}"#, lat, lon)
}

const BIND_ADDR: &str = "192.168.1.68:9001";

/// Handles WebSocket connections and broadcasts GPS data to each connected client.
#[tracing::instrument]
async fn start_websocket_server(mut rx: broadcast::Receiver<String>) {
    tracing::debug!("Before TCP bind...");
    let listener = TcpListener::bind(BIND_ADDR).await.unwrap();
    tracing::info!("WebSocket server now running at ws://localhost:9001");

    // grab the TCP stream from the listener
    tracing::debug!("Before stream listener accept...");
    let (stream, _addr) = listener.accept().await.unwrap();
    tracing::debug!("Stream listener accepted!");

    // make it into a websocket stream
    tracing::debug!("Before accepting WebSocket stream...");
    let ws_stream = accept_async(stream).await.unwrap();
    tracing::debug!("WebSocket stream accepted!");

    // and immediately split it into a reader + writer.
    //
    // since we're only writing, we can safely ignore the reader :)
    tracing::debug!("Splitting WS sink into reader + writer...");
    let (mut write, _read) = ws_stream.split();
    tracing::debug!("WebSocket split completed!");

    loop {
        // while we get data from the gps subscriber stream...
        while let Ok(gps_data) = rx.recv().await {
            // write it onto the websocket
            match write.send(Message::Text(Utf8Bytes::from(gps_data))).await {
                Ok(_unit) => tracing::debug!("Sent message successfully!"),
                Err(e) => {
                    tracing::error!("Client disconnected! err: {e}");
                    break;
                }
            }
        }
    }
}
