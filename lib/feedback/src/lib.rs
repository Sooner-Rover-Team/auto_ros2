//! # Feedback
//!
//! A library crate that encodes/decodes messages to or from the Rover.
//!
//! ## Protocol
//!
//! The protocol is as follows:
//!
//! ### Wheels
//!
//! [u8; 9]: [0x01 (wheels subsystem), 0x01 (wheels part), ]
//!
//! subsystem byte, part byte (optional); etc.

pub mod checksum;
pub mod error;
pub mod parse;
pub mod send;

pub mod prelude {
    pub use super::send::RoverController;
}

/// For the Rover, the Wheels struct represents the current state of each of the six wheels.
/// Each `wheelx` value is a u8, with the neutral position being 126.
#[cfg_attr(feature = "python", pyo3::pyclass)]
#[derive(Debug, Clone, Copy)]
pub struct Wheels {
    pub left: u8,
    pub right: u8,
    pub checksum: u8,
}

impl Wheels {
    pub const SUBSYSTEM_BYTE: u8 = 0x01;
    pub const PART_BYTE: u8 = 0x01;

    /// The motor value at which a motor isn't moving.
    pub const NEURTAL_SPEED: u8 = 126;

    /// Creates a new `Wheels`.
    pub const fn new(left: u8, right: u8) -> Self {
        Self {
            left,
            right,

            // see electrical ebox teensy code
            checksum: 255_u8.wrapping_add(left.wrapping_add(right)),
        }
    }
}

/// The flashing LED on the top of the Rover
#[cfg_attr(feature = "python", pyo3::pyclass)]
#[derive(Debug, Clone, Copy)]
pub struct Led {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl Led {
    pub const SUBSYSTEM_BYTE: u8 = 0x01;
    pub const PART_BYTE: u8 = 0x02;
}

/// The little robotic arm on the sticking out of the Rover
/// old capstooOOOone
#[cfg_attr(feature = "python", pyo3::pyclass)]
#[derive(Debug, Clone, Copy)]
pub struct Arm {
    pub bicep: u8,
    pub forearm: u8,
    pub base: u8,
    pub wrist_pitch: u8,
    pub wrist_roll: u8,
    pub claw: u8,
    pub checksum: u8,
}

impl Arm {
    pub const SUBSYSTEM_BYTE: u8 = 0x02;
}

/// The science package on the Rover, including the utilities needed to perform
/// field experiments.
#[cfg_attr(feature = "python", pyo3::pyclass)]
#[derive(Debug, Clone, Copy)]
pub struct Science {
    big_actuator: u8,
    drill: u8,
    small_actuator: u8,
    test_tubes: u8,
    camera_servo: u8,
    checksum: u8,
}

impl Science {
    pub const SUBSYSTEM_BYTE: u8 = 0x03;
}

/// Data about the three paired sensors. Sent from the E-box microcontroller
/// to the Jetson Orin Nano.
#[cfg_attr(feature = "python", pyo3::pyclass)]
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Imu {
    // accel
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,

    // gyro
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,

    // compass
    pub compass_x: f32,
    pub compass_y: f32,
    pub compass_z: f32,

    // temperature
    pub temp_c: f32,
}

impl Imu {
    pub const SUBSYSTEM_BYTE: u8 = 0x04;
}

/// Python stuff.
#[cfg(feature = "python")]
mod python {
    use pyo3::prelude::*;

    use crate::{Arm, Imu, Led, Science, Wheels};

    #[pymethods]
    impl Wheels {
        /// Creates a new `Wheels` object from the given values. Unchecked.
        #[new]
        pub const fn pynew(
            wheel0: u8,
            wheel1: u8,
            wheel2: u8,
            wheel3: u8,
            wheel4: u8,
            wheel5: u8,
            checksum: u8,
        ) -> Self {
            Self {
                wheel0,
                wheel1,
                wheel2,
                wheel3,
                wheel4,
                wheel5,
                checksum,
            }
        }

        fn __str__(&self) -> PyResult<String> {
            Ok(format!("{:?}", self))
        }
    }

    #[pymethods]
    impl Led {
        fn __str__(&self) -> PyResult<String> {
            Ok(format!("{:?}", self))
        }
    }

    #[pymethods]
    impl Arm {
        fn __str__(&self) -> PyResult<String> {
            Ok(format!("{:?}", self))
        }
    }

    #[pymethods]
    impl Science {
        fn __str__(&self) -> PyResult<String> {
            Ok(format!("{:?}", self))
        }
    }

    #[pymethods]
    impl Imu {
        fn __str__(&self) -> PyResult<String> {
            Ok(format!("{:?}", self))
        }
    }

    #[pymodule]
    fn feedback(m: &Bound<'_, PyModule>) -> PyResult<()> {
        m.add_class::<Wheels>()?;
        m.add_class::<Led>()?;
        m.add_class::<Arm>()?;
        m.add_class::<Science>()?;
        Ok(())
    }
}
