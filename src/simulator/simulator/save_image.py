import signal
import sys
from dataclasses import dataclass

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image
from typing_extensions import override

# these both need to be changed.
#
# - ("/sensors/mono_image", "bgr8")
# - ("/sensors/depth_image", "32FC1")
TOPIC: str = "/sensors/depth_image"
COLOR_CODE: str = "32FC1"


@dataclass(kw_only=True)
class ImageSaver(Node):
    bridge: CvBridge
    subscription: Subscription
    saved: bool = False

    def __init__(self):
        super().__init__("image_saver")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, TOPIC, self.image_callback, 10
        )
        print(f"waiting to read from topic `{TOPIC}`...")

    def image_callback(self, msg):
        if not self.saved:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, COLOR_CODE)
                cv2.imwrite("camera_image.png", cv_image)
                print("Image saved as camera_image.png")
                self.saved = True

                rclpy.try_shutdown()
                sys.exit(0)
            except Exception as e:
                print(f"failed to convert (w/ cv_bridge). err: {e}")

    @override
    def __hash__(self) -> int:
        return super().__hash__()


def exit_handler(_handler: int, _):
    rclpy.try_shutdown()
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()

    _ = signal.signal(signal.SIGINT, exit_handler)
    _ = signal.signal(signal.SIGTERM, exit_handler)

    try:
        rclpy.spin(image_saver)
    except KeyboardInterrupt:
        pass
    finally:
        image_saver.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
