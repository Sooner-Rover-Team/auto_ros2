from geographic_msgs.msg import GeoPoint

from navigator_node.coords import calc_angle_to_target


def make_geopoint(lat: float, lon: float, alt: float) -> GeoPoint:
    """constructs a new geopoint. the args are in meters"""
    pt: GeoPoint = GeoPoint()
    pt.latitude = lat
    pt.longitude = lon
    pt.altitude = alt

    return pt


# takes long, lat (x,y)
def calc_angle(
    target_longitude: float,
    target_latitude: float,
) -> float:
    """calc_angle_to_target_from_robot"""

    """base helper function for tests"""
    # coordinate of the robot
    robot_coord: GeoPoint = make_geopoint(0.0, 0.0, 0.0)

    # coordinate of some {location, tag, object}
    # Get the angle to a coordinate that is in front of the robot
    target_coord: GeoPoint = make_geopoint(
        target_latitude, target_longitude, 0.0
    )

    # a target in front of rover will have an angle diff. of 0 deg.
    return calc_angle_to_target(target_coord, robot_coord)


def test_origin_lon_diff():
    """
    at the origin, putting a target right in front of the Rover should
    result in an angle of .
    """
    # a target in front of rover will have an angle diff. of 0 deg.
    assert calc_angle(0.0, 0.0001) == 0


def test_two_lon_diff():
    """same as above, but far from the origin"""
    # a target in front of rover will have an angle diff. of 0 deg.
    assert calc_angle(0.0, 1.0) == 0


def test_left_angle():
    """Test that a destination to our east will be -90 degrees from the rover"""
    assert calc_angle(-1.0, 0.0) == -90.0


def test_right_angle():
    """Test that a destination to our west will be 90 degrees from the rover"""
    assert calc_angle(1.0, 0.0) == 90.0
    pass


def test_behind_angle():
    """Checking angle when a target is directly behind (south)"""
    assert calc_angle(0.0, -1.0) == -180.0
