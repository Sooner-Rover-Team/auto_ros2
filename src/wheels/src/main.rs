use std::{
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
    time::Duration,
};

use feedback::{prelude::RoverController, Wheels};
use safe_drive::{
    context::Context, logger::Logger, msg::common_interfaces::geometry_msgs, node::Node, pr_debug,
    pr_error, pr_info, topic::subscriber::Subscriber,
};
use tokio::sync::{
    mpsc::{unbounded_channel, UnboundedReceiver, UnboundedSender},
    RwLock,
};

/// The name of this Node.
const WHEELS_NODE_NAME: &str = "wheels_node";

/// The `ros2_control` topic to which we subscribe.
///
/// It provides
/// [`geometry_msgs::msg::Twist`](https://docs.ros.org/en/humble/p/geometry_msgs/msg/Twist.html)
// messages that we convert into wheel speeds.
const CMD_VEL_TOPIC_NAME: &str = "/cmd_vel";

#[tokio::main(flavor = "multi_thread")]
async fn main() {
    // enable logging to terminal
    tracing_subscriber::fmt()
        .pretty()
        .with_max_level(tracing::Level::INFO)
        .init();

    // init ros2 context, which is just the DDS middleware
    let ctx: Arc<Context> = Context::new().expect("init ros 2 context");

    // create the wheels node
    let node: Arc<Node> = ctx
        .create_node(WHEELS_NODE_NAME, Some("/"), Default::default())
        .inspect_err(|e| tracing::error!("Failed to create `{WHEELS_NODE_NAME}`! err: {e}"))
        .expect("node creation");

    // init logger and tell user we're online
    let logger: Arc<Logger> = Arc::new(Logger::new(WHEELS_NODE_NAME));
    pr_info!(logger, "The `{WHEELS_NODE_NAME}` is now online!");

    // create a subscription to the `/cmd_vel` topic.
    //
    // that's a topic that provides `sensor_msgs::msg::Twist` messages! we'll
    // need to convert them into actual wheel speeds before we can do anything
    // else...
    //
    // for more info, see:
    // https://docs.ros.org/en/humble/p/geometry_msgs/msg/Twist.html
    pr_debug!(logger, "Creating subscriber for `{CMD_VEL_TOPIC_NAME}`!");
    let subscriber = node
        .create_subscriber::<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC_NAME, None)
        .unwrap();

    // alright, let's connect to the Ebox to send the wheels...
    //
    // we'll need the ip and port for the responsible microcontroller
    let (ip, port): (Ipv4Addr, u16) = (Ipv4Addr::new(192, 168, 1, 102), 5002);

    // with those, we can now send stuff to the Ebox microcontroller.
    //
    // note: this isn't a TCP connection, so we're kinda just hoping that we're
    // talking to someone on the other end.
    //
    // it's easier for Electrical, though, so this behavior is expected and
    // (generally) wanted ;D
    pr_info!(logger, "Creating `RoverController`...");
    let controller = RoverController::new(IpAddr::V4(ip), port, 0)
        .await
        .expect("Failed to create RoverController");
    pr_info!(logger, "`RoverController` created!");

    // let's also make a channel so we can talk between two tasks.
    //
    // having two tasks will keep our wheel speeds in order, even if things get
    // a little hectic...
    //
    // it'll also prevent us from blocking on finding the microcontroller
    // within our ROS 2 subscription task - it's in the other one!
    let (tx, rx) = unbounded_channel();
    let shared_stop_bool: Arc<RwLock<bool>> = Arc::new(RwLock::new(false));

    // spawn the subscription task onto another thread and convert messages
    // when we get them!
    pr_info!(logger, "Spawning wheel subscriber task...");
    let t = tokio::task::spawn(wheels_subscriber_task(
        subscriber,
        tx,
        Arc::clone(&logger),
        Arc::clone(&shared_stop_bool),
    ));
    pr_info!(logger, "Wheel subscriber task spawned successfully.");

    // we'll just await the sender task on the main thread - it should run
    // forever, after all.
    //
    // the `ctrl_c` task is there to stop the other task when we're told to
    // exit. we'll tell the user when that happens below
    tokio::select![
        _ = tokio::signal::ctrl_c() => {
            *shared_stop_bool.write().await = true;
        },
        _ = wheels_sender_task(
            controller,
            rx,
            Arc::clone(&logger),
            Arc::clone(&shared_stop_bool)
        ) => (),
        _ = t => (),
    ];

    // say goodbye
    pr_info!(logger, "Asked to shutdown! Doing so gracefully. Goodbye!");
}

/// This async function listens for messages on `/cmd_vel`.
//
// When it sees one, it'll convert it into wheel speeds that Electrical is
// expecting to receive...
//
// Then, it'll send that info to the Ebox communication task.
//
// - `subscriber`: a ROS 2 sub to the `Twist` messages
// - `tx`: transfers converted wheel speeds to the Ebox comms task
// - `logger`: it... uh... logs. (to rosout)
// - `shared_stop_bool`: tells us when to stop this task
async fn wheels_subscriber_task(
    mut subscriber: Subscriber<geometry_msgs::msg::Twist>,
    tx: UnboundedSender<Wheels>,
    logger: Arc<Logger>,
    shared_stop_bool: Arc<RwLock<bool>>,
) {
    pr_info!(logger, "The `/cmd_vel` subscriber is now active.");

    // constantly listen for wheel twist messages on `/cmd_vel`...
    loop {
        // check if we need to stop
        if *shared_stop_bool.read().await {
            return;
        }

        // grab a message from the subscriber
        let maybe_message = subscriber.recv().await;

        // only keep listening if it's `Result::Ok` (as opposed to a `Result::Err`)
        let message = match maybe_message {
            // it's `Ok`, so just steal the value it's holding
            Ok(m) => m,

            // it's an `Err`! report it and look again...
            Err(e) => {
                // this might be an error because we requested that the node
                // stops.
                //
                // in that case, we won't error, and instead, we'll just return
                // happily :)
                if *shared_stop_bool.read().await {
                    return;
                }

                pr_error!(logger, "Failed to get `Twist` msg. err: {e}");
                continue;
            }
        };

        // alright, if we reached here, we now have a message! let's grab the
        // useful bits.
        //
        // IMPORTANT: we need to mind the ROS 2 "axis orientation" spec:
        // https://www.ros.org/reps/rep-0103.html#axis-orientation
        //
        // - pos. x means to drive forward
        // - pos. y is left of the Rover
        let forward_backward = message.linear.x;
        let left_right = message.angular.z;

        // linear simply decides how fast we're going.
        //
        // note that we're sending `i8` here, as that's "really" what
        // Electrical is expecting.
        //
        // reminder: `i8` has a range of [-128, 127].
        let base_speed: i8 = convert_speed(forward_backward);

        // angular decides how much we scale the speed on each side.
        let (left_speed, right_speed) = create_wheel_speeds(base_speed, left_right);

        // now, we can send that to the Ebox
        // make message into something we can send to the microcontroller.
        let wheels = Wheels::new(left_speed, right_speed);

        // send that to the other task.
        //
        // note: this will never block
        _ = tx.send(wheels).inspect_err(|e| {
            pr_error!(
                logger,
                "Failed to `tx` wheels message onto sender task. err: {e}"
            )
        });
    }
}

/// Talks to the Ebox.
///
/// - `controller`: connects to the Ebox to control the wheels
/// - `rx`: gets messages from the other task
/// - `logger`: logs (to rosout)
/// - `shared_stop_bool`: tells us when to return
async fn wheels_sender_task(
    controller: RoverController,
    mut rx: UnboundedReceiver<Wheels>,
    logger: Arc<Logger>,
    shared_stop_bool: Arc<RwLock<bool>>,
) {
    pr_info!(logger, "Ebox connection task is now active.");

    // this while loop will continue running until we the `UnboundedSender`
    // goes offline.
    //
    // the `rx.recv()` await point block blocks forever - we don't do any work
    // on this task while waiting
    while let Some(msg) = rx.recv().await {
        // check if we need to stop
        if *shared_stop_bool.read().await {
            return;
        }

        // let's try sending it to the rover controller.
        //
        // we'll time out if that doesn't work
        match tokio::time::timeout(Duration::from_secs(2), controller.send_wheels(&msg)).await {
            Ok(send_result) => {
                if let Err(e) = send_result {
                    pr_error!(logger, "Failed to send wheel speeds to Ebox! err: {e}");
                }
            }

            Err(e) => pr_error!(
                logger,
                "Timed out while sending wheel speeds to Ebox! (timed out after {e} seconds)"
            ),
        }
    }
}

/// This small helper function converts a speed value encoded as [-1.0, 1.0]
/// into the values that Electrical is expecting to receive.
//
// That means we map it with respect to the following:
//
// - values <= -1.0 become -128.
// - values >= 1.0 are now 127
// - everything between is mapped linearly
fn convert_speed(speed: f64) -> i8 {
    // clamp the speed value to the expected range
    let speed: f64 = speed.clamp(-1.0, 1.0);

    // scale the value by the number of bits on each 'half' (128)
    const SCALE: f64 = 128.0;
    let scaled_speed: f64 = speed * SCALE;

    // round that scaled value, then store it in a larger `i16`.
    //
    // we can do this since we've now got
    let rounded_speed: f64 = scaled_speed.round().clamp(i8::MIN as f64, i8::MAX as f64);

    // finally, return that value with a rounded value
    rounded_speed as i8
}

/// Returns a pair of `(left, right)` wheel speeds.
fn create_wheel_speeds(base_speed: i8, left_right: f64) -> (u8, u8) {
    // we can actually re-use the speed conversion for the `left_right` calc;
    // it's got the same exact [-1.0, 1.0] range and linear property
    const TURN_SCALE: f64 = 1.0; // arbitrary; feel free to play w/ it
    let turn_amount: i8 = convert_speed(left_right * TURN_SCALE);

    // this is the world's smallest differential drive implementation.
    //
    // for more info about what that means, see:
    // https://en.wikipedia.org/wiki/Differential_wheeled_robot
    let left = base_speed.saturating_sub(turn_amount);
    let right = base_speed.saturating_add(turn_amount);

    // we'll encode these "in reverse" according to the format.
    //
    // they need to be in `u8`, as we send em like that.
    //
    // Rust casts `i8 -> u8` while keeping bits **exactly the same**, so we
    // can just use an `as` cast and be done with it. see:
    // https://doc.rust-lang.org/reference/expressions/operator-expr.html#r-expr.as.numeric

    let f = |s: i8| -> u8 {
        if s > 0 {
            return (s as i16 + 128).min(255) as u8;
        }
        (s as i16 + 127).max(0) as u8
    };

    // return what we got
    (f(left), f(right))
}
