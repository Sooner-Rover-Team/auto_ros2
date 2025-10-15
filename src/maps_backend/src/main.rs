//! # Maps backend (`maps_node`)
//!
//! This node subscribes to the `/sensors/gps` topic and listens to its
//! messages.
//!
//! When it finds a GPS message, it then publishes it over a WebSocket to the
//! frontend.

use core::fmt::Write as _;
use std::sync::Arc;

use safe_drive::{
    context::Context, msg::common_interfaces::sensor_msgs, node::Node, qos::Profile,
    topic::subscriber::Subscriber,
};
use sensor_msgs::msg::NavSatFix;

use futures_util::{SinkExt, StreamExt};
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

    // Set up broadcast channel to share GPS data with WebSocket clients.
    //
    // Note that `16` is the buffer size.
    tracing::debug!("Making thread broadcast channel...");
    let (tx, rx): (broadcast::Sender<String>, broadcast::Receiver<String>) =
        broadcast::channel::<String>(16);
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
        // holds the json text we create
        let mut json = String::new();

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

                // update `json` with the new JSON text
                create_json_coordinate_pair(&mut json, lat, lon);

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
fn create_json_coordinate_pair(s: &mut String, lat: f64, lon: f64) {
    s.clear(); // clear the string

    // push all this stuff to the string in order
    s.push_str(r#"{"lat": "#); // {"lat":
    write!(s, "{lat:?}").unwrap(); //  {"lat": A.Bbbbbbb
    s.push_str(r#", "lon": "#); // {"lat": A.Bbbbbbb, "lon":
    write!(s, "{lon:?}").unwrap(); // {"lat": A.Bbbbbbb, "lon": C.Dddddd
    s.push('}'); // DONE! {"lat": A.Bbbbbbb, "lon": C.Dddddd}

    // since `s` is `&mut`, we're modifying it instead of returning a new
    // string.
    //
    // this is moderately faster, which could be important at comp
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

#[cfg(test)]
mod tests {
    use core::{
        assert_eq,
        f64::consts::PI,
        sync::atomic::{AtomicBool, Ordering},
        time::Duration,
    };
    use std::{sync::Arc, time::SystemTime};

    use safe_drive::{
        context::Context,
        msg::{
            builtin_interfaces::UnsafeTime,
            common_interfaces::{
                sensor_msgs::msg::{NavSatFix, NavSatStatus},
                std_msgs::msg::Header,
            },
            RosString,
        },
        node::Node,
        qos::Profile,
        topic::subscriber::Subscriber,
    };
    use tokio::{sync::broadcast, time::Timeout};

    /// Checks that we only create JSON floats -- not numbers that can be
    /// parsed as integers!
    #[test]
    fn creates_valid_json_floats() {
        // init logging
        _ = tracing_subscriber::fmt()
            .with_max_level(tracing::Level::TRACE)
            .try_init();

        // use these pairs
        #[expect(
            clippy::type_complexity,
            reason = "types are ugly, but who cares? it's a test man"
        )]
        const COORD_RESULT_PAIRS: &[((f64, f64), (&str, &str))] = &[
            ((1.0, 2.0), ("1.0", "2.0")),
            ((PI, -43.67219), ("3.141592653589793", "-43.67219")),
            ((0_f64, 0_f64), ("0.0", "0.0")),
        ];

        // check that each pair matches expectations
        for ((lat, lon), (lat_str, lon_str)) in COORD_RESULT_PAIRS {
            let mut as_json: String = String::new();
            super::create_json_coordinate_pair(&mut as_json, *lat, *lon);

            let split = as_json.split(", ").collect::<Vec<_>>();
            let [lat_json, lon_json] = split.as_slice() else {
                panic!();
            };

            assert_eq!(lat_json, &format!(r#"{{"lat": {lat_str}"#).as_str());
            assert_eq!(lon_json, &format!(r#""lon": {lon_str}}}"#).as_str());
        }
    }

    /// Ensures that we receive ROS 2 messages correctly.
    ///
    /// This test:
    ///
    /// - Creates the `maps_node`
    /// - Creates a fake "sensors" publisher node
    /// - Fails if the `maps_node` fails to get all messages within time limit
    ///
    /// Note that this is a `tokio` test, so it's async. That means we have
    /// full access to its async runtime, and tasks can run in the background!
    #[tokio::test]
    async fn recv_ros2_messages() {
        // init logging
        _ = tracing_subscriber::fmt()
            .with_max_level(tracing::Level::TRACE)
            .try_init();

        // create shared bool (so we can wait for both threads to be ready)
        let is_ready: Arc<AtomicBool> = Arc::new(AtomicBool::new(false));

        // create channels to get json output
        let (tx, mut rx): (broadcast::Sender<String>, broadcast::Receiver<String>) =
            broadcast::channel::<String>(16);

        let ctx: Arc<Context> = Context::new().unwrap();

        // create a lil sender node to test it out
        const MAX_MESSAGES: i32 = 10;
        let ctx_2 = Arc::clone(&ctx);
        let is_ready_2: Arc<AtomicBool> = Arc::clone(&is_ready); // ugly b/c of `move` rules
        std::thread::spawn(move || {
            let sender_node: Arc<Node> = ctx_2
                .create_node(
                    "maps_backend_sender_test_node",
                    Some("/"),
                    Default::default(),
                )
                .unwrap();

            let gps_publisher = sender_node
                .create_publisher::<NavSatFix>(
                    "/sensors/gps/rust_test",
                    Some(Profile::sensor_data()),
                )
                .unwrap();

            // wait until the other node is up
            while !is_ready_2.load(Ordering::SeqCst) {
                std::thread::sleep(Duration::from_millis(5));
            }
            std::thread::sleep(Duration::from_millis(500));

            for i in 0..MAX_MESSAGES {
                let timestamp: Duration = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap();

                if timestamp.as_secs() > (i32::MAX as u64) {
                    panic!("year 2038 bug detected (or system time is very wrong)");
                }

                let (lat, lon): (f64, f64) = (i as f64 + 0.5, i as f64 + 2.3);

                let n: NavSatFix = NavSatFix {
                    header: Header {
                        stamp: UnsafeTime {
                            sec: timestamp.as_secs() as i32,
                            nanosec: timestamp.subsec_nanos(),
                        },
                        frame_id: RosString::new("im a gps frame").unwrap(),
                    },
                    status: NavSatStatus::new().unwrap(),
                    latitude: lat,
                    longitude: lon,
                    altitude: i as f64 + 624.0153,
                    position_covariance: [0_f64; 9],
                    position_covariance_type: 0,
                };

                tracing::info!("sender node: loop {i}: sending coord: ({lat:?}, {lon:?})");
                gps_publisher.send(&n).expect("send coord");
                std::thread::sleep(Duration::from_millis(20));
            }
        });

        // then, create the maps node
        let node: Arc<Node> = ctx
            .create_node("maps_backend_test_node", Some("/"), Default::default())
            .unwrap();
        let gps_subscriber: Subscriber<NavSatFix> = node
            .create_subscriber::<NavSatFix>("/sensors/gps/rust_test", Some(Profile::sensor_data()))
            .unwrap();

        // spawn the maps node in the background.
        //
        // this is the node we're testing.
        tokio::task::spawn(super::handle_ros2_messages(gps_subscriber, tx));
        is_ready.store(true, Ordering::SeqCst);

        // helper func for below...
        let helper_require_ten_msgs_within_time: Timeout<_> = tokio::time::timeout(
            Duration::from_millis(1000),
            tokio::task::spawn(async move {
                let mut counter = 0;
                loop {
                    let _ = rx.recv().await.expect("should get json successfully");
                    counter += 1;

                    // stop looking if we're @ max messages
                    if counter >= MAX_MESSAGES {
                        return;
                    }
                }
            }),
        );

        // ensure we get ten messages within required time.
        //
        // if not, panic (which fails the test)
        match helper_require_ten_msgs_within_time.await {
            Ok(_) => {
                tracing::info!("Test completed successfully!");
            }
            Err(e) => {
                panic!("Reached timeout, or got err! err: {e}!");
            }
        };
    }
}
