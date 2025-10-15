from dataclasses import dataclass

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose


@dataclass
class FoundMarkerInformation:
    """
    Information about which markers we've found and where they are.

    Before using anything in here, check that you aren't re-sending old
    information.
    """

    marker_ids: list[int] | None = None
    """
    A list of marker IDs we found on the most recent refresh.
    """

    marker_poses: list[Pose] | None = None
    """
    The poses for each marker we've detected.

    These are in relation to the `base_link` (Rover).
    """

    time_last_image_arrived: Time | None = None
    """
    When the last image arrived.

    You can check this to avoid re-sending old data to the client(s).
    """
