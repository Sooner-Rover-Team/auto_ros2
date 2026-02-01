#![doc = include_str!("../README.md")]

use libsbf::reader::SbfReader;
use std::{
    fs::File,
    path::PathBuf,
    time::{Duration, Instant},
};

use error::GpsConnectionError;

pub mod error;

/// The fastest possible update time for the GPS.
///
/// This is 1/20th of a second.
pub const BEST_UPDATE_TIME: Duration = Duration::from_millis(1000 / 20);

/// A representation of the GPS.
pub struct Gps {
    /// A reader for the GPS.
    ///
    /// Uses the Septentrio Binary Format (SBF).
    reader: SbfReader<File>,
}

impl Gps {
    /// Attempts to initialize the GPS.
    ///
    /// ## Errors
    ///
    /// This constructor will error if the serial path given does not exist.
    #[tracing::instrument]
    pub fn new(gps_path: PathBuf) -> Result<Self, GpsConnectionError> {
        // try opening the file at that path
        let file: File = std::fs::File::open(&gps_path).map_err(|e| {
            // log an error
            tracing::error!(
                "Failed to open GPS at the given serial path. Path: `{}`, Error: {}",
                gps_path.to_string_lossy(),
                e
            );

            // map to the connection error ty
            GpsConnectionError::FileNotFound(gps_path.clone())
        })?;

        tracing::debug!(
            "Connected to GPS over serial! Path: `{}`",
            gps_path.to_string_lossy()
        );

        let reader: SbfReader<File> = SbfReader::new(file);

        Ok(Self { reader })
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

        // grab data until we have the correct message types
        let mut last_time_got_data: Instant = Instant::now();
        let mut last_time_warned: Instant = Instant::now();
        loop {
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

            for msg in self.reader {
                let msg = match msg {
                    Ok(m) => m,
                    Err(e) => {
                        tracing::error!("Failed to get messages from the GPS reader! err: {e}");
                        return None;
                    }
                };

                match msg {
                    libsbf::Messages::MeasExtra(meas_extra) => todo!(),
                    libsbf::Messages::GALNav(galnav) => todo!(),
                    libsbf::Messages::PVTGeodetic(pvtgeodetic) => todo!(),
                    libsbf::Messages::ReceiverStatus(receiver_status) => todo!(),
                    libsbf::Messages::Commands(commands) => todo!(),
                    libsbf::Messages::GEORawL1(georaw_l1) => todo!(),
                    libsbf::Messages::MeasEpoch(meas_epoch) => todo!(),
                    libsbf::Messages::GALIon(galion) => todo!(),
                    libsbf::Messages::GALUtc(galutc) => todo!(),
                    libsbf::Messages::GALGstGps(galgst_gps) => todo!(),
                    libsbf::Messages::GPSCNav(gpscnav) => todo!(),
                    libsbf::Messages::INSSupport(inssupport) => todo!(),
                    libsbf::Messages::Meas3Ranges(meas3_ranges) => todo!(),
                    libsbf::Messages::Meas3Doppler(meas3_doppler) => todo!(),
                    libsbf::Messages::BDSIon(bdsion) => todo!(),
                    libsbf::Messages::ExtSensorStatus(ext_sensor_status) => todo!(),
                    libsbf::Messages::INSNavGeod(insnav_geod) => todo!(),
                    libsbf::Messages::VelSensorSetup(vel_sensor_setup) => todo!(),
                    libsbf::Messages::AttEuler(att_euler) => todo!(),
                    libsbf::Messages::AttCovEuler(att_cov_euler) => todo!(),
                    libsbf::Messages::DiffCorrIn(diff_corr_in) => todo!(),
                    libsbf::Messages::ExtSensorMeas(ext_sensor_meas) => todo!(),
                    libsbf::Messages::QualityInd(quality_ind) => todo!(),
                    libsbf::Messages::ExtSensorInfo(ext_sensor_info) => todo!(),
                    libsbf::Messages::ImuSetup(imu_setup) => todo!(),
                    libsbf::Messages::ReceiverSetup(receiver_setup) => todo!(),
                    libsbf::Messages::GEONav(geonav) => todo!(),
                    libsbf::Messages::GPSIon(gpsion) => todo!(),
                    libsbf::Messages::GPSNav(gpsnav) => todo!(),
                    libsbf::Messages::GPSUtc(gpsutc) => todo!(),
                    libsbf::Messages::PosCovGeodetic(pos_cov_geodetic) => todo!(),
                    libsbf::Messages::Unsupported(_) => todo!(),
                }
            }

            // remember that we got some data
            tracing::trace!("got new message from GPS!");
            last_time_got_data = Instant::now();

            // we'll try to parse each message now.
            //
            // some of them are useless, and others are essential. we gotta
            // check each to find out!
            for msg in all_parsed {
                // this might be the message we want, but we gotta check (below)
                let maybe_good_msg = match msg {
                    Ok(frame) => frame,
                    Err(e) => {
                        tracing::warn!("Failed to parse this message! err: {e}");
                        return Err(GpsReadError::ParseFailed(e));
                    }
                };

                // we'll take any of the following gps messages:
                //
                // - MsgPosLlh
                // - MsgPosLlhCov
                match maybe_good_msg {
                    // if we have covariances, we can just use those!
                    sbp::Sbp::MsgPosLlhCov(_) => {
                        if let Some(info) = parsing::msg_pos_llh_cov(&maybe_good_msg) {
                            return Ok(info);
                        }
                    }

                    // without covariances, we'll just say we don't have them
                    sbp::Sbp::MsgPosLlh(_) => {
                        if let Some(info) = parsing::msg_pos_llh(&maybe_good_msg) {
                            return Ok(info);
                        }
                    }

                    // other variants are untested.
                    //
                    // TODO: add additonal fallbacks if needed!
                    _ => (),
                };
            }
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
/// something useful for maintainers of the `sensors::sensors_node`.
///
/// ## Why are there so many parsers?
///
/// Despite being so similar, the Swift GPS reports these as different message
/// types, so we have to parse each individually, with respect to each
/// variant's special properties.
///
/// We need to check for all of them due to our need for a fallback. At URC
/// 2025, we noticed that our GPS was only providing `MsgPosLlh`, despite
/// getting `MsgPosLlhCov` back in Oklahoma. By parsing both, we can be more
/// certain that we're parsing useful messages if we're receiving them - even
/// if they don't contain _everything_ we'd want... in a perfect world.
///
/// tl;dr: Swift has a lotta different messages, and they change depending on
/// time, region, and luck.
mod parsing {
    use crate::{Coordinate, FixStatus, GpsInfo, Height, TimeOfWeek};

    /// Parses from a `MsgPosLlhCov`.
    ///
    /// This is our preferred message types...
    #[tracing::instrument]
    pub fn msg_pos_llh_cov(raw_msg: &sbp::Sbp) -> Option<GpsInfo> {
        // ensure we got the right message type
        let sbp::Sbp::MsgPosLlhCov(m) = raw_msg else {
            tracing::error!("Provided wrong message type!");
            return None;
        };

        // grab the fix mode
        let fix_mode = match m.fix_mode() {
            Ok(fm) => fm,
            Err(e) => {
                tracing::error!("Failed to get fix mode from SwiftNav `MsgPosLlh`! err code: {e}");
                return None;
            }
        };

        // and convert that into a fix status
        use sbp::messages::navigation::msg_pos_llh_cov::FixMode;
        let fix_status = match fix_mode {
            FixMode::Invalid | FixMode::DeadReckoning => FixStatus::Invalid,
            FixMode::SinglePointPosition => FixStatus::SinglePoint,
            FixMode::SbasPosition => FixStatus::Sbas,
            FixMode::DifferentialGnss | FixMode::FloatRtk | FixMode::FixedRtk => {
                FixStatus::GroundBased
            }
        };
        // create the covariance.
        //
        // note: ROS 2 wants `ENU`, but Swift provides `NED`.
        // we'll move things around and flip down -> up...
        let position_covariance: [f64; 9] = [
            // east
            m.cov_e_e as f64,
            m.cov_n_e as f64,
            -m.cov_e_d as f64,
            // north
            m.cov_n_e as f64,
            m.cov_n_n as f64,
            -m.cov_n_d as f64,
            // up
            -m.cov_e_d as f64,
            -m.cov_n_d as f64,
            m.cov_d_d as f64,
        ];

        // create info from the basic stuff
        let info = GpsInfo {
            coord: Coordinate {
                lat: m.lat,
                lon: m.lon,
            },
            height: Height(m.height),
            tow: TimeOfWeek(m.tow),
            fix_status,
            covariance: position_covariance,
        };

        return Some(info);
    }

    /// Parses from a `MsgPosLlh`, which doesn't have covariances. That means
    /// the GPS isn't reporting how sure it is about its answer!
    ///
    /// We prefer position messages that provide that info instead, but this is
    /// here as fallback.
    #[tracing::instrument]
    pub fn msg_pos_llh(raw_msg: &sbp::Sbp) -> Option<GpsInfo> {
        // ensure we got the right message type
        let sbp::Sbp::MsgPosLlh(m) = raw_msg else {
            tracing::error!("Provided wrong message type!");
            return None;
        };

        // grab the fix mode
        let fix_mode = match m.fix_mode() {
            Ok(fm) => fm,
            Err(e) => {
                tracing::error!("Failed to get fix mode from SwiftNav `MsgPosLlh`! err code: {e}");
                return None;
            }
        };

        // and convert that into a fix status
        use sbp::messages::navigation::msg_pos_llh::FixMode;
        let fix_status = match fix_mode {
            FixMode::Invalid | FixMode::DeadReckoning => FixStatus::Invalid,
            FixMode::SinglePointPosition => FixStatus::SinglePoint,
            FixMode::SbasPosition => FixStatus::Sbas,
            FixMode::DifferentialGnss | FixMode::FloatRtk | FixMode::FixedRtk => {
                FixStatus::GroundBased
            }
        };

        // create info from the basic stuff.
        //
        // we'll report `covariance` as all zeroes
        let info = GpsInfo {
            coord: Coordinate {
                lat: m.lat,
                lon: m.lon,
            },
            height: Height(m.height),
            tow: TimeOfWeek(m.tow),
            fix_status,
            covariance: [0_f64; 9],
        };

        return Some(info);
    }
}
