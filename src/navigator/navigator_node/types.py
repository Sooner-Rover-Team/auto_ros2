from dataclasses import dataclass
from enum import Enum

from geographic_msgs.msg import GeoPoint


class NavigationMode(Enum):
    GPS = 0
    ARUCO = 1
    OBJECT_DETECTION = 2


@dataclass(kw_only=True)
class NavigationParameters:
    coord: GeoPoint
    """
    The coordinate that either is our goal (if `mode` is `GPS`), or is some
    point around an ArUco marker or an object.
    """

    aruco_marker_id: int | None = None
    """
    We're searching for a marker with this ID.

    ArUco markers have IDs defined by a dictionary. URC uses 4x4_50.
    https://docs.opencv.org/3.4/d9/d6a/group__aruco.html#ggac84398a9ed9dd01306592dd616c2c975ada8e830ff0024e839e93c01f5fed0c55
    """

    mode: NavigationMode
    """
    Indicates what the Navigator is doing.

    For example, if we're given `NavigationMode::GPS`, we'll navigate to the given GPS
    coordinate and stop when we're there.
    """
