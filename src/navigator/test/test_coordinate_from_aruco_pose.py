import math

import pytest
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped

from navigator_node.coords import (
    coordinate_from_aruco_pose,
    dist_m_between_coords,
)


def make_geopoint(latitude: float, longitude: float, altitude: float) -> GeoPoint:
    pt: GeoPoint = GeoPoint()
    pt.latitude = latitude
    pt.longitude = longitude
    pt.altitude = altitude

    return pt


def make_pose_stamped(x: float, y: float, z: float) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose


def test_coordinate_from_aruco_pose_zero_offset_returns_same_lat_lon_and_alt():
    current = make_geopoint(35.0, -97.0, 123.4)
    marker_pose = make_pose_stamped(0.0, 0.0, 0.0)

    output = coordinate_from_aruco_pose(current, marker_pose)

    assert output.latitude == pytest.approx(current.latitude)
    assert output.longitude == pytest.approx(current.longitude)
    assert output.altitude == pytest.approx(current.altitude)


@pytest.mark.parametrize(
    ("north_offset_m", "east_offset_m", "up_offset_m"),
    [
        (10.0, 0.0, 0.0),  # North
        (0.0, 10.0, 0.0),  # East
        (-10.0, 0.0, 0.0),  # South
        (0.0, -10.0, 0.0),  # West
        (3.0, 4.0, 2.5),  # Diagonal + altitude
        (-6.0, 8.0, -1.0),  # Mixed signs
    ],
)
def test_coordinate_from_aruco_pose_preserves_meter_distance_and_altitude(
    north_offset_m: float, east_offset_m: float, up_offset_m: float
):
    current = make_geopoint(35.0, -97.0, 50.0)
    marker_pose = make_pose_stamped(north_offset_m, east_offset_m, up_offset_m)

    output = coordinate_from_aruco_pose(current, marker_pose)

    expected_distance_m = math.hypot(north_offset_m, east_offset_m)
    horizontal_distance_m = dist_m_between_coords(current, output)

    # Geodesic destination should preserve the rover-local horizontal displacement magnitude.
    assert horizontal_distance_m == pytest.approx(
        expected_distance_m, rel=1e-6, abs=1e-6
    )
    # Vertical offset is mapped directly to GeoPoint altitude delta.
    assert output.altitude == pytest.approx(current.altitude + up_offset_m)


def test_coordinate_from_aruco_pose_axes_map_to_cardinal_directions():
    current = make_geopoint(35.0, -97.0, 0.0)

    north = coordinate_from_aruco_pose(current, make_pose_stamped(5.0, 0.0, 0.0))
    east = coordinate_from_aruco_pose(current, make_pose_stamped(0.0, 5.0, 0.0))
    south = coordinate_from_aruco_pose(current, make_pose_stamped(-5.0, 0.0, 0.0))
    west = coordinate_from_aruco_pose(current, make_pose_stamped(0.0, -5.0, 0.0))

    assert north.latitude > current.latitude
    assert east.longitude > current.longitude
    assert south.latitude < current.latitude
    assert west.longitude < current.longitude
