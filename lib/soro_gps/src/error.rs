use std::path::PathBuf;

/// An error that can occur when connecting to the GPS.
#[derive(Clone, Debug)]
pub enum GpsConnectionError {
    /// The GPS' serial output file representation was not found at the given
    /// location.
    FileNotFound(
        /// The given location.
        PathBuf,
    ),
}

impl core::error::Error for GpsConnectionError {}

impl core::fmt::Display for GpsConnectionError {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        match self {
            GpsConnectionError::FileNotFound(serial_path) => write!(
                f,
                "The GPS' serial output file representation was not found at \
                the given location: `{}`",
                serial_path.to_string_lossy()
            ),
        }
    }
}
