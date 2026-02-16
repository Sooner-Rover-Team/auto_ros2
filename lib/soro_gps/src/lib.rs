#![doc = include_str!("../README.md")]

use libsbf::{parser::SbfParser, Messages};
use std::{
    fs::File,
    io::{ErrorKind, Read},
    os::unix::fs::FileTypeExt,
    path::PathBuf,
    time::{Duration, Instant},
};

use error::GpsConnectionError;

pub mod error;

/// The fastest possible update time for the GPS.
///
/// This is 1/20th of a second.
pub const BEST_UPDATE_TIME: Duration = Duration::from_millis(1000 / 20);
/// Maximum allowed TOW delta (in ms) when pairing PVT with covariance.
const COVARIANCE_TOW_TOLERANCE_MS: u32 = 1000;

/// A representation of the GPS.
pub struct Gps {
    /// Underlying byte source (serial TTY or a sample file).
    source: Box<dyn Read + Send>,

    /// Streaming parser for Septentrio Binary Format (SBF).
    parser: SbfParser,

    /// Last known covariance block keyed by TOW.
    latest_covariance_by_tow: Option<(u32, [f64; 9])>,
}

impl Gps {
    /// Attempts to initialize the GPS.
    ///
    /// ## Errors
    ///
    /// This constructor will error if the serial path given does not exist.
    #[tracing::instrument]
    pub fn new(gps_path: PathBuf) -> Result<Self, GpsConnectionError> {
        let gps_path_str = gps_path.to_string_lossy().to_string();
        if gps_path_str.contains("Septentrio_USB_Device") && gps_path_str.contains("-if04") {
            tracing::warn!(
                "Connecting to `{}`. On many Septentrio mosaic-Go setups, \
                binary SBF output is exposed on `...-if02`. If `get()` blocks \
                forever, switch to the `if02` interface.",
                gps_path_str
            );
        }

        let metadata = std::fs::metadata(&gps_path).map_err(|e| {
            tracing::error!(
                "Failed to access GPS source metadata. Path: `{}`, Error: {}",
                gps_path.to_string_lossy(),
                e
            );
            GpsConnectionError::FileNotFound(gps_path.clone())
        })?;

        let source: Box<dyn Read + Send> = if metadata.file_type().is_char_device() {
            const USB_CDC_BAUD: u32 = 115_200;
            const READ_TIMEOUT: Duration = Duration::from_millis(500);

            let port = serialport::new(gps_path_str.clone(), USB_CDC_BAUD)
                .timeout(READ_TIMEOUT)
                .open_native()
                .map_err(|e| {
                    tracing::error!(
                        "Failed to open GPS serial device. Path: `{}`, Error: {}",
                        gps_path.to_string_lossy(),
                        e
                    );
                    GpsConnectionError::FailedToOpen(gps_path.clone(), e.to_string())
                })?;

            tracing::debug!(
                "Connected to GPS over serial! Path: `{}`, timeout={:?}",
                gps_path.to_string_lossy(),
                READ_TIMEOUT
            );
            Box::new(port)
        } else {
            let file: File = std::fs::File::open(&gps_path).map_err(|e| {
                tracing::error!(
                    "Failed to open GPS source file. Path: `{}`, Error: {}",
                    gps_path.to_string_lossy(),
                    e
                );
                GpsConnectionError::FailedToOpen(gps_path.clone(), e.to_string())
            })?;

            tracing::debug!(
                "Connected to GPS sample file source. Path: `{}`",
                gps_path.to_string_lossy()
            );
            Box::new(file)
        };

        Ok(Self {
            source,
            parser: SbfParser::new(),
            latest_covariance_by_tow: None,
        })
    }

    /// Attempts to get the required GPS message.
    ///
    /// This can run forever! Run it on a background task if that won't work
    /// for your usage.
    #[tracing::instrument(skip(self))]
    pub fn get(&mut self) -> Option<GpsInfo> {
        tracing::trace!(
            "Attempting to get a GPS message. If this message \
            shows up without progress, no messages are being recv'd."
        );

        // Grab data until we have a usable `PVTGeodetic`, ideally paired with
        // a matching `PosCovGeodetic` from the same TOW.
        //
        // We enforce a timeout so malformed/non-SBF streams do not block
        // forever inside parsing.
        const MAX_WAIT_FOR_FIX: Duration = Duration::from_millis(1500);
        const READ_BUFFER_SIZE: usize = 8192;

        let start_time: Instant = Instant::now();
        let mut last_time_got_data: Instant = Instant::now();
        let mut last_time_warned: Instant = Instant::now();
        let mut buffer = [0_u8; READ_BUFFER_SIZE];

        loop {
            if start_time.elapsed() > MAX_WAIT_FOR_FIX {
                tracing::warn!(
                    "Timed out waiting for a usable PVTGeodetic fix after {:.2} seconds. \
                    Please check that the stream is configured to output SBF; \
                    also, ensure that the GPS antenna has a view of the sky.",
                    start_time.elapsed().as_secs_f32()
                );
                return None;
            }

            // Drain any already-buffered parser data first.
            if let Some(msg) = self.parser.consume(&[]) {
                if let Some(info) = self.handle_message(msg) {
                    return Some(info);
                }
                continue;
            }

            // if we haven't collected data in a while, we'll warn the user!
            if last_time_got_data.elapsed() > Duration::from_millis(1500)
                && last_time_warned.elapsed() > Duration::from_millis(1500)
            {
                tracing::warn!(
                    "Haven't heard from the GPS in {:.2} seconds!",
                    last_time_got_data.elapsed().as_secs_f32()
                );
                last_time_warned = Instant::now();
            }

            let bytes_read = match self.source.read(&mut buffer) {
                Ok(0) => {
                    tracing::warn!("GPS stream ended.");
                    return None;
                }
                Ok(n) => n,
                Err(e) if matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::WouldBlock) => {
                    continue;
                }
                Err(e) => {
                    tracing::error!("Failed to read bytes from GPS source! err: {e}");
                    return None;
                }
            };

            tracing::trace!("Read {bytes_read} bytes from GPS source.");
            last_time_got_data = Instant::now();

            if let Some(msg) = self.parser.consume(&buffer[..bytes_read]) {
                if let Some(info) = self.handle_message(msg) {
                    return Some(info);
                }
            }
        }
    }

    fn handle_message(&mut self, msg: Messages) -> Option<GpsInfo> {
        tracing::trace!("Handling message...");

        match msg {
            Messages::PosCovGeodetic(pos_cov_geodetic) => {
                tracing::debug!("Got new `PosCovGeodetic` (position info)!");
                self.latest_covariance_by_tow =
                    parsing::pos_cov_geodetic_covariance(&pos_cov_geodetic);
                None
            }
            Messages::PVTGeodetic(pvt_geodetic) => {
                tracing::debug!("Got new `PVTGeodetic` (covariances)!");
                let covariance = self.covariance_for_tow(pvt_geodetic.tow);

                parsing::pvt_geodetic_to_info(&pvt_geodetic, covariance)
            }
            _ => None,
        }
    }

    fn covariance_for_tow(&self, pvt_tow: Option<u32>) -> [f64; 9] {
        match (pvt_tow, self.latest_covariance_by_tow.as_ref()) {
            (Some(tow), Some((cov_tow, cov))) if tow == *cov_tow => *cov,
            (Some(tow), Some((cov_tow, cov)))
                if tow.abs_diff(*cov_tow) <= COVARIANCE_TOW_TOLERANCE_MS =>
            {
                tracing::trace!(
                    "Using near-match covariance: pvt_tow={} cov_tow={} (delta={}ms)",
                    tow,
                    cov_tow,
                    tow.abs_diff(*cov_tow)
                );
                *cov
            }
            _ => [0.0_f64; 9],
        }
    }
}

/// Information from the GPS.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct GpsInfo {
    pub coord: Coordinate,
    pub height: Height,
    pub tow: TimeOfWeek,
    pub covariance: [f64; 9],
    pub fix_status: FixStatus,
}

impl core::fmt::Display for GpsInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "GpsInfo {{ coord(lat: {}, lon: {}), height: {} m above wsg84, tow: {} ms }}",
            self.coord.lat, self.coord.lon, self.height.0, self.tow.0
        )
    }
}

/// Some coordinate returned from the GPS.
///
/// The returned values are in degrees.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Coordinate {
    pub lat: f64,
    pub lon: f64,
}

/// Height, with the unit being "meters above a WGS84 ellipsoid".
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Height(pub f64);

/// Error in millimeters. Inside the `gps` library, this is calculated as the
/// average of the vertical and horizontal 'error'.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct ErrorInMm(pub f64);

/// The GPS time of week, given from the satellite. This is measured in
/// milliseconds since the start of this GNSS week.
///
/// For additional information, see
/// [the NOAA documentation](https://www.ngs.noaa.gov/CORS/Gpscal.shtml).
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct TimeOfWeek(pub u32);

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub enum FixStatus {
    Invalid,
    SinglePoint,
    Sbas,
    GroundBased,
}

/// Everything in this module is related to parsing the GPS messages into
/// something useful for the `sensors::sensors_node`.
///
/// We derive position from `PVTGeodetic` and enrich it with covariance from
/// `PosCovGeodetic` when both blocks share the same time-of-week.
mod parsing {
    use crate::{Coordinate, FixStatus, GpsInfo, Height, TimeOfWeek};
    use libsbf::{PVTGeodetic, PosCovGeodetic, PvtMode};

    /// Parses covariance from `PosCovGeodetic`.
    #[tracing::instrument]
    pub fn pos_cov_geodetic_covariance(msg: &PosCovGeodetic) -> Option<(u32, [f64; 9])> {
        let tow = msg.tow?;
        let (
            Some(cov_latlat),
            Some(cov_lonlon),
            Some(cov_hgthgt),
            Some(cov_latlon),
            Some(cov_lathgt),
            Some(cov_lonhgt),
        ) = (
            msg.cov_latlat,
            msg.cov_lonlon,
            msg.cov_hgthgt,
            msg.cov_latlon,
            msg.cov_lathgt,
            msg.cov_lonhgt,
        )
        else {
            tracing::trace!("PosCovGeodetic missing covariance values for tow={tow}");
            return None;
        };

        // PosCovGeodetic is in (lat, lon, hgt) axes. ROS NavSatFix expects
        // ENU covariance order [E, N, U] in row-major form.
        //
        // Axis mapping:
        // E <- lon
        // N <- lat
        // U <- hgt
        let position_covariance = [
            cov_lonlon as f64,
            cov_latlon as f64,
            cov_lonhgt as f64,
            cov_latlon as f64,
            cov_latlat as f64,
            cov_lathgt as f64,
            cov_lonhgt as f64,
            cov_lathgt as f64,
            cov_hgthgt as f64,
        ];

        Some((tow, position_covariance))
    }

    /// Parses a `PVTGeodetic` into `GpsInfo`.
    #[tracing::instrument]
    pub fn pvt_geodetic_to_info(msg: &PVTGeodetic, covariance: [f64; 9]) -> Option<GpsInfo> {
        let (Some(tow), Some(lat), Some(lon), Some(height)) =
            (msg.tow, msg.latitude, msg.longitude, msg.height)
        else {
            tracing::warn!(
                "PVTGeodetic doesn't have position information. Ignoring... \
                (tow: `{:?}`, lat: `{:?}`, lon: `{:?}`, height: `{:?}`)",
                msg.tow,
                msg.latitude,
                msg.longitude,
                msg.height
            );
            return None;
        };

        Some(GpsInfo {
            // SBF PVTGeodetic latitude/longitude are in radians.
            coord: Coordinate {
                lat: lat.to_degrees(),
                lon: lon.to_degrees(),
            },
            height: Height(height),
            tow: TimeOfWeek(tow),
            fix_status: mode_to_fix_status(msg.pvt_mode()),
            covariance,
        })
    }

    fn mode_to_fix_status(mode: PvtMode) -> FixStatus {
        match mode {
            PvtMode::NoPvt => FixStatus::Invalid,
            PvtMode::StandAlone | PvtMode::FixedLocation => FixStatus::SinglePoint,
            PvtMode::Sbas => FixStatus::Sbas,
            PvtMode::Differential
            | PvtMode::RtkFixed
            | PvtMode::RtkFloat
            | PvtMode::MovingBaseRtkFixed
            | PvtMode::MovingBaseRtkFloat
            | PvtMode::Ppp => FixStatus::GroundBased,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{parsing, FixStatus, Gps};
    use libsbf::{reader::SbfReader, Messages, PVTGeodetic, PosCovGeodetic};
    use std::fs::File;

    fn data_file(file_name: &str) -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("data_files")
            .join(file_name)
    }

    #[test]
    fn inside_bay_contains_pvt_and_poscov_blocks() {
        let file = File::open(data_file("inside_bay.sbf")).expect("sample file should open");
        let reader = SbfReader::new(file);

        let mut saw_pvt = false;
        let mut saw_pos_cov = false;
        for msg in reader {
            let msg = msg.expect("inside_bay SBF should decode");
            match msg {
                Messages::PVTGeodetic(_) => saw_pvt = true,
                Messages::PosCovGeodetic(_) => saw_pos_cov = true,
                _ => {}
            }

            if saw_pvt && saw_pos_cov {
                break;
            }
        }

        assert!(saw_pvt, "inside_bay.sbf should contain PVTGeodetic");
        assert!(saw_pos_cov, "inside_bay.sbf should contain PosCovGeodetic");
    }

    #[test]
    fn parses_outside_bay_sample() {
        let mut gps = Gps::new(data_file("outside_the_bay.sbf")).expect("sample file should open");
        let info = gps
            .get()
            .expect("should parse at least one PVTGeodetic message");
        assert!(info.coord.lat.is_finite());
        assert!(info.coord.lon.is_finite());
    }

    #[test]
    fn minimal_sbf_regression_expected_values() {
        let file = File::open(data_file("minimal_sbf.sbf")).expect("sample file should open");
        let reader = SbfReader::new(file);

        let mut pvt_geodetic: Option<PVTGeodetic> = None;
        let mut pos_cov_geodetic: Option<PosCovGeodetic> = None;

        for msg in reader {
            let msg = msg.expect("minimal_sbf SBF should decode");
            match msg {
                Messages::PVTGeodetic(pvt) if pvt_geodetic.is_none() => pvt_geodetic = Some(pvt),
                Messages::PosCovGeodetic(pos_cov) if pos_cov_geodetic.is_none() => {
                    pos_cov_geodetic = Some(pos_cov)
                }
                _ => {}
            }

            if pvt_geodetic.is_some() && pos_cov_geodetic.is_some() {
                break;
            }
        }

        let pvt_geodetic = pvt_geodetic.expect("minimal_sbf.sbf should contain PVTGeodetic");
        let pos_cov_geodetic =
            pos_cov_geodetic.expect("minimal_sbf.sbf should contain PosCovGeodetic");

        assert_eq!(pvt_geodetic.tow, Some(162480000));
        assert_close(
            pvt_geodetic.latitude.expect("lat should exist"),
            0.6145361740716572,
            1e-12,
        );
        assert_close(
            pvt_geodetic.longitude.expect("lon should exist"),
            -1.7006749045222338,
            1e-12,
        );
        assert_close(
            pvt_geodetic.height.expect("height should exist"),
            326.3521656619435,
            1e-12,
        );

        assert_eq!(pos_cov_geodetic.tow, Some(162480000));
        assert_close(
            f64::from(
                pos_cov_geodetic
                    .cov_latlat
                    .expect("cov_latlat should exist"),
            ),
            1.4662505,
            5e-7,
        );
        assert_close(
            f64::from(
                pos_cov_geodetic
                    .cov_lonlon
                    .expect("cov_lonlon should exist"),
            ),
            6.332064,
            5e-7,
        );
        assert_close(
            f64::from(
                pos_cov_geodetic
                    .cov_hgthgt
                    .expect("cov_hgthgt should exist"),
            ),
            7.9160724,
            5e-7,
        );
        assert_close(
            f64::from(
                pos_cov_geodetic
                    .cov_latlon
                    .expect("cov_latlon should exist"),
            ),
            -1.0395553,
            5e-7,
        );
        assert_close(
            f64::from(
                pos_cov_geodetic
                    .cov_lathgt
                    .expect("cov_lathgt should exist"),
            ),
            -1.793132,
            5e-7,
        );
        assert_close(
            f64::from(
                pos_cov_geodetic
                    .cov_lonhgt
                    .expect("cov_lonhgt should exist"),
            ),
            4.2639227,
            5e-7,
        );

        let expected_covariance = [
            6.332064151763916,
            -1.039555311203003,
            4.263922691345215,
            -1.039555311203003,
            1.4662505388259888,
            -1.793131947517395,
            4.263922691345215,
            -1.793131947517395,
            7.916072368621826,
        ];
        let (cov_tow, covariance) = parsing::pos_cov_geodetic_covariance(&pos_cov_geodetic)
            .expect("minimal_sbf.sbf covariance should be complete");
        assert_eq!(cov_tow, 162480000);
        for (actual, expected) in covariance.iter().zip(expected_covariance.iter()) {
            assert_close(*actual, *expected, 1e-12);
        }

        let info = parsing::pvt_geodetic_to_info(&pvt_geodetic, covariance)
            .expect("PVT should parse into GpsInfo");
        assert_close(info.coord.lat, 35.21032913242285, 1e-12);
        assert_close(info.coord.lon, -97.44149435293824, 1e-12);
        assert_close(info.height.0, 326.3521656619435, 1e-12);
        assert_eq!(info.tow.0, 162480000);
        assert_eq!(info.fix_status, FixStatus::SinglePoint);
        for (actual, expected) in info.covariance.iter().zip(expected_covariance.iter()) {
            assert_close(*actual, *expected, 1e-12);
        }
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        let delta = (actual - expected).abs();
        assert!(
            delta <= tolerance,
            "expected {expected} +/- {tolerance}, got {actual} (delta={delta})"
        );
    }
}
