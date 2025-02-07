//! Contains a test mocking the microcontroller and ensuring that lights
//! 'should' flash.
//!
//! Note that most Rust projects combine tests into one file. However, we tried
//! that, and doing so here made the logic difficult to follow.

use std::{
    net::Ipv4Addr,
    time::{Duration, Instant},
};

use feedback::send::RoverController;
use futures_lite::FutureExt;
use ros2_client::{
    log::LogLevel, rosout, AService, Context, Name, NodeName, NodeOptions, ServiceMapping,
    ServiceTypeName,
};

use crate::{logic, LightsRequest, LightsResponse};

const FAKE_MICROCONTROLLER_PORT: u16 = 9003;

/// Checks that we can handle requests and send to microcontroller.
#[tokio::test]
#[tracing::instrument]
async fn request_handled_ok() {
    tracing_subscriber::fmt()
        .with_env_filter(
            "RUST_LOG=ERROR,lights_node=DEBUG,feedback=DEBUG,ros2_client[rosout_raw]=DEBUG",
        )
        .init();

    let ctx = Context::new().unwrap();
    let mut node = logic::create_node(&ctx);

    let server = node
        .create_server::<AService<LightsRequest, LightsResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/", "lights_service").unwrap(),
            &ServiceTypeName::new("lights_node", "lights"),
            logic::qos(),
            logic::qos(),
        )
        .unwrap();

    logic::spin(&mut node);

    // wait for messages on a faked microcontroller
    tracing::info!("spawning fake microcontroller");
    let ebox_task = tokio::task::spawn(fake_microcontroller_task());

    // make a client to send stuff to the server
    tracing::info!("spawning client node");
    tokio::task::spawn(client_node_task(ctx));

    // look for messages, forever
    tracing::info!("spawning server task");
    let controller =
        RoverController::new(Ipv4Addr::UNSPECIFIED.into(), FAKE_MICROCONTROLLER_PORT, 0)
            .await
            .unwrap();
    tokio::task::spawn(logic::request_handler(server, node, controller));

    // join on the microcontroller task.
    //
    // that'll make us either finish correctly or panic
    tracing::info!("waiting for ebox task...");
    ebox_task.await.unwrap();
    tracing::info!("ebox task returned correctly!");
}

/// Makes a client node to test the server node.
///
/// Ya'know. The one defined in `main.rs`. :D
#[tracing::instrument(skip_all)]
async fn client_node_task(ctx: Context) {
    let mut friend_node = ctx
        .new_node(
            NodeName::new("/rustdds", "friend_node").unwrap(),
            NodeOptions::new().enable_rosout(true),
        )
        .unwrap();

    rosout!(friend_node, LogLevel::Debug, "client node created!");

    let client = friend_node
        .create_client::<AService<LightsRequest, LightsResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/", "lights_service").unwrap(),
            &ServiceTypeName::new("lights_node", "lights"),
            logic::qos(),
            logic::qos(),
        )
        .unwrap();

    logic::spin(&mut friend_node);
    tokio::time::sleep(Duration::from_millis(100)).await;

    tracing::info!("Sending request...");
    let lights_response = client
        .async_send_request(LightsRequest {
            red: 255,
            green: 0,
            blue: 255,
            flashing: false,
        })
        .await
        .unwrap();
    tracing::info!("resp: {lights_response:?}");
}

/// Pretends to be a microcontroller. Used to check that we've actually sent
/// the light struct as expected.
async fn fake_microcontroller_task() {
    // potentially confusing: split this into functions inside the function
    // to easily have future types.
    //
    // these are in a race, and if we get a msg on the socket, we win the race!
    // oh i mean. we pass the test! anyways...
    //
    // try to recv for max of 10s
    //
    // if we don't get anything by then, give up.
    futures::future::select(give_up_future().boxed(), wait_for_packet().boxed()).await;

    /// gives up after ten seconds
    #[tracing::instrument]
    async fn give_up_future() {
        let start_time = Instant::now();

        loop {
            if start_time.elapsed() > Duration::from_secs(10) {
                panic!("the node never sent anything! hit timeout.");
            }

            tokio::time::sleep(Duration::from_millis(200)).await;
        }
    }

    /// waits for some packet, trying to 'beat' the give up future.
    #[tracing::instrument]
    async fn wait_for_packet() {
        let socket = tokio::net::UdpSocket::bind((Ipv4Addr::LOCALHOST, FAKE_MICROCONTROLLER_PORT))
            .await
            .unwrap();
        let mut buf = vec![0x0; 24];

        // wait for one message to be sent.
        //
        // if it's an error, we panic
        tracing::debug!("waiting for message");
        let Ok(bytes_recvd) = socket.recv(&mut buf).await else {
            panic!("got an error when trying to recv bytes!");
        };
        eprintln!("got message! {:?}", &buf[..5]);

        // expect purple
        assert_eq!(bytes_recvd, 5);
        assert_eq!(&buf[..5], &[1, 2, 255, 0, 255]);
    }
}
