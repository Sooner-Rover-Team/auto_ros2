use feedback::{prelude::RoverController, Wheels};
use futures_lite::StreamExt as _;
use ros2_client::{
    log::LogLevel, ros2::QosPolicyBuilder, rosout, Context, MessageTypeName, Name, Node, NodeName,
    NodeOptions, Publisher, Subscription,
};
use std::{net::Ipv4Addr, sync::Arc, time::Duration};
use tokio::sync::Mutex;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct WheelsMessage {
    pub left_wheels: u8,
    pub right_wheels: u8,
}
impl ros2_client::Message for WheelsMessage {}

#[tokio::main(flavor = "multi_thread")]
async fn main() {
    // enable logging to terminal
    tracing_subscriber::fmt()
        .pretty()
        .with_env_filter("RUST_LOG=error,wheels_node=DEBUG,feedback=DEBUG")
        .init();

    // init ros2 context, which is just the DDS middleware
    let ros2_context = Context::new().expect("init ros 2 context");

    // create the wheels node
    let node = Arc::new(Mutex::new(create_node(&ros2_context)));
    tracing::info!("wheels node is online!");

    // make sthe wheels topic
    let wheels_topic = {
        let node_locked = node.lock().await;
        let topic = node_locked
            .create_topic(
                &Name::new("/", "wheels").unwrap(),
                MessageTypeName::new("wheels_node", "Wheels"),
                &ros2_client::DEFAULT_SUBSCRIPTION_QOS,
            )
            .expect("create wheels topic");

        topic
    };

    // subscribe to wheels topic
    let subscriber: Subscription<WheelsMessage> = {
        let mut node_locked = node.lock().await;
        node_locked
            .create_subscription(
                &wheels_topic,
                Some(qos()), // Apply the Quality of Service settings.
            )
            .expect("create subscription")
    };

    // microcontroller that is responsible for controlling wheels
    // needs the microcontrollers' ip address and port
    let (ebox_ip, ebox_port, port) = (Ipv4Addr::new(192, 168, 1, 102), 5002, 6660);
    let controller = RoverController::new(ebox_ip.into(), ebox_port, port)
        .await
        .expect("Failed to create RoverController");
    tracing::info!("Created Rover Controller.");

    // processes the recieved commands and sends the commands to the microcontroller
    let join_handle = tokio::task::spawn(async move {
        let subscriber = subscriber; // move sub to task's stack
        let mut stream = Box::pin(subscriber.async_stream());

        tracing::info!("in the task :D");
        while let Some(resp) = stream.next().await {
            tracing::info!("got a msg!");

            let msg = match resp {
                Ok(msg) => msg.0,
                Err(e) => {
                    tracing::error!("Failed to get next topic message! err: {e}");
                    continue;
                }
            };

            let checksum = (msg
                .left_wheels
                .wrapping_add(msg.left_wheels.wrapping_mul(3))
                .wrapping_add(msg.right_wheels.wrapping_mul(3)));

            let wheels = Wheels {
                wheel0: msg.left_wheels,
                wheel1: msg.left_wheels,
                wheel2: msg.left_wheels,
                wheel3: msg.right_wheels,
                wheel4: msg.right_wheels,
                wheel5: msg.right_wheels,
                checksum,
            };

            let _result = controller
                .send_wheels(&wheels)
                .await
                .inspect_err(|e| tracing::error!("Failed to send wheel speeds! err: {e}"));
        }
    });

    // make the node do stuff
    {
        let mut node_locked = node.lock().await;
        spin(&mut node_locked);
    }

    friend_node(&ros2_context).await;

    if let Err(e) = join_handle.await {
        eprintln!("Error in join_handle task: {:?}", e);
    }

    // What is thine purpose?
    //tokio::time::sleep(Duration::from_secs(17)).await;

    tracing::info!("wheels node is shutting down...");
}

/// Creates the `wheels_node`.
fn create_node(ctx: &Context) -> Node {
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

// converts the incoming message into the correct byte array based on feedback
#[tracing::instrument]
fn parse_wheels_msg(msg: &str) -> Option<Wheels> {
    let bytes = msg.as_bytes();

    let parsed_msg_res = feedback::parse::parse(bytes)
        .inspect_err(|e| tracing::debug!("Couldn't parse message! err: {e}"))
        .ok();

    if let Some(parsed_msg) = parsed_msg_res {
        if let feedback::parse::Message::Wheels(wheels) = parsed_msg {
            tracing::debug!("Got a wheels message! {wheels:#?}");
            return Some(wheels);
        }
    }

    None
}

#[tracing::instrument(skip_all)]
pub async fn friend_node(ctx: &Context) {
    let mut friend_node = ctx
        .new_node(
            NodeName::new("/", "wheels_friend_node").unwrap(),
            NodeOptions::new().enable_rosout(true),
        )
        .unwrap();

    let topic = friend_node
        .create_topic(
            &Name::new("/", "wheels").unwrap(),
            MessageTypeName::new("wheels_node", "Wheels"),
            &ros2_client::DEFAULT_SUBSCRIPTION_QOS,
        )
        .expect("create wheels topic");

    let publisher: Publisher<WheelsMessage> =
        friend_node.create_publisher(&topic, Some(qos())).unwrap();

    spin(&mut friend_node);

    // pub in a loop lol
    tokio::spawn(async move {
        loop {
            tracing::debug!("Sending wheels 25% message...");
            publisher
                .publish(WheelsMessage {
                    left_wheels: 200,
                    right_wheels: 200,
                })
                .unwrap();
            tracing::debug!("Sent 25%!");

            tokio::time::sleep(Duration::from_millis(2000)).await;

            tracing::debug!("Sending wheels 0% message...");
            publisher
                .publish(WheelsMessage {
                    left_wheels: 126,
                    right_wheels: 126,
                })
                .unwrap();
            tracing::debug!("Sent 0%!");
            tokio::time::sleep(Duration::from_millis(2000)).await;
        }
    });
}
