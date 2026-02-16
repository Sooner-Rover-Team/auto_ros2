import xml.etree.ElementTree as ET
from pathlib import Path


def _robot_tree() -> ET.Element:
    urdf_xacro = (
        Path(__file__).resolve().parents[1] / "resource" / "rover.urdf.xacro.xml"
    )
    return ET.parse(urdf_xacro).getroot()


def test_lidar_frames_attached_to_chassis_tf_tree() -> None:
    root: ET.Element[str] = _robot_tree()

    links: set[str] = {link.attrib["name"] for link in root.findall("link")}
    assert "unilidar_imu" in links
    assert "unilidar_lidar" in links

    parent_by_child: dict[str, str] = {}
    for joint in root.findall("joint"):
        parent: ET.Element[str] | None = joint.find("parent")
        child: ET.Element[str] | None = joint.find("child")
        if parent is None or child is None:
            continue
        parent_by_child[child.attrib["link"]] = parent.attrib["link"]

    assert parent_by_child.get("unilidar_imu") == "chassis"
    assert parent_by_child.get("unilidar_lidar") == "unilidar_imu"
