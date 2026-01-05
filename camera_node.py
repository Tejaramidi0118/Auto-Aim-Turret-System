import os
import time
import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Defaults (allow overriding from env)
CAM_DEVICE = os.getenv("CAM_DEVICE", "/dev/video0")
try:
    FRAME_WIDTH = int(os.getenv("FRAME_WIDTH", "640"))
    FRAME_HEIGHT = int(os.getenv("FRAME_HEIGHT", "480"))
    LOOP_HZ = float(os.getenv("LOOP_HZ", "20.0"))
    OPEN_RETRIES = int(os.getenv("OPEN_RETRIES", "6"))
    OPEN_DELAY = float(os.getenv("OPEN_DELAY", "0.5"))
except Exception:
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    LOOP_HZ = 20.0
    OPEN_RETRIES = 6
    OPEN_DELAY = 0.5


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        qos = rclpy.qos.QoSProfile(depth=10)
        self.pub = self.create_publisher(Image, "/camera/image_raw", qos)
        self.bridge = CvBridge()
        self.cap = None

        # Resolve CAM_DEVICE: allow integer index strings
        cam_open_target = CAM_DEVICE
        try:
            if isinstance(CAM_DEVICE, str) and CAM_DEVICE.strip().isdigit():
                cam_open_target = int(CAM_DEVICE.strip())
        except Exception:
            cam_open_target = CAM_DEVICE

        opened = False
        last_exc = None
        for attempt in range(OPEN_RETRIES):
            try:
                if isinstance(cam_open_target, str):
                    self.get_logger().info(f"Attempting to open camera device {cam_open_target} (attempt {attempt+1})")
                    cap = cv2.VideoCapture(cam_open_target, cv2.CAP_V4L2)
                else:
                    self.get_logger().info(f"Attempting to open camera index {cam_open_target} with V4L2 backend (attempt {attempt+1})")
                    cap = cv2.VideoCapture(int(cam_open_target), cv2.CAP_V4L2)

                try:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
                except Exception:
                    pass

                time.sleep(0.08)

                if cap is not None and cap.isOpened():
                    self.cap = cap
                    opened = True
                    self.get_logger().info(f"Opened camera {cam_open_target} on attempt {attempt+1}")
                    break
                else:
                    try:
                        cap.release()
                    except Exception:
                        pass
                    self.get_logger().warning(f"Camera open attempt {attempt+1} failed")
            except Exception as e:
                last_exc = e
                self.get_logger().warning(f"Camera open attempt {attempt+1} exception: {e}")
            time.sleep(OPEN_DELAY)

        if not opened:
            self.get_logger().fatal(f"Cannot open camera device {CAM_DEVICE} after {OPEN_RETRIES} attempts")
            if last_exc:
                self.get_logger().debug(traceback.format_exc())
            raise SystemExit("camera not available")

        period = 1.0 / max(0.001, LOOP_HZ)
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info("Camera node started, publishing /camera/image_raw")

    def timer_cb(self):
        if self.cap is None:
            self.get_logger().warning("Timer fired but camera handle is None")
            return

        try:
            ret, frame = self.cap.read()
        except Exception as e:
            self.get_logger().warning(f"Camera read exception: {e}")
            return

        if not ret or frame is None:
            self.get_logger().warning("Camera read failed")
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().warning(f"cv_bridge failure: {e}")
        except Exception as e:
            self.get_logger().warning(f"Unexpected error converting/publishing frame: {e}")

    def destroy_node(self):
        try:
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
        except Exception:
            pass
        try:
            super().destroy_node()
        except Exception:
            pass


def main(args=None):
    try:
        rclpy.init(args=args)
    except Exception:
        pass

    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except SystemExit:
        raise
    except Exception as e:
        if rclpy.ok() and node is not None:
            node.get_logger().fatal(f"Unhandled exception in camera_node: {e}")
        else:
            print(f"FATAL: Unhandled exception in camera_node: {e}")
        raise
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
