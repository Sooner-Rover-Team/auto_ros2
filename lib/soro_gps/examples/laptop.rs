//! This example should work on a laptop/desktop computer.
//!
//! I'm using it now to test the GPS connection without any ROS 2 stuff to mess
//! with anything.

use soro_gps::Gps;

fn main() {
    // IMPORTANT: change this to the GPS' serial connection file.
    //
    // on Windows, you might be outta luck, but macOS and Linux should allow
    // grabbing this with: `ls /dev | grep ACM`
    const DEVICE_PATH: &str = "TODO";

    // then, "make a connection" by reading from that file!
    let mut gps: Gps = match Gps::new(DEVICE_PATH.into()) {
        Ok(gps) => gps,
        Err(e) => {
            tracing::error!("GPS connection error: {e}");
            panic!();
        }
    };

    // continuously grab GPS information
    loop {
        match gps.get() {
            Some(gps_info) => {
                tracing::info!("New GPS message: {gps_info:?}")
            }

            Option::None => {
                tracing::error!("GPS disconnected! Waiting 2s and trying again...");
                std::thread::sleep(core::time::Duration::from_millis(2_000));
                continue;
            }
        }
    }
}
