import math
import time
import logging

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

# ---------- CONFIG ----------
# HSV thresholds (example: orange-ish / skin tone). Tune to your target.
LOWER_HSV = np.array([0, 60, 60])
UPPER_HSV = np.array([25, 255, 255])

MIN_CONTOUR_AREA = 1500    # minimal area to consider valid detection
STREAM_W = 1024            # used only for visualization text
STREAM_H = 576
LOOP_HZ = 20.0

# virtual distance estimation constants (from your previous script)
A = 0.012636680507237852
B = -2.710541724316941
C = 182.62076069382988

# offsets to apply to turret (adjust if needed)
OFFSET_X = 12
OFFSET_Y = 0
OFFSET_Z = 7

# ----------------------------

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# import Angles helper: try relative then package-qualified; if failure, keep None
try:
    from .Angles import turret as TurretAngles
except Exception:
    try:
        from auto_turret.Angles import turret as TurretAngles
    except Exception as e:
        TurretAngles = None
        logging.critical(f"Cannot import Angles.py: {e}")

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info("Detection node starting")

        # publishers
        self.pub_img = self.create_publisher(Image, '/camera/image_annotated', 10)
        self.pub_pt = self.create_publisher(Point, '/target/coords', 10)

        # bridge
        self.bridge = CvBridge()

        # subscribe to camera raw images
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb_image, 10)

        # internal state
        self.last_time = time.time()
        self.frame_idx = 0

        self.get_logger().info("Detection node started")

    def cb_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f"cv_bridge conversion failed: {e}")
            return

        self.frame_idx += 1
        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            cx = x + cw // 2
            cy = y + ch // 2
            if area > best_area:
                best_area = area
                best = (x, y, x + cw, y + ch, cx, cy, area)

        if best is not None:
            x1, y1, x2, y2, cx, cy, area = best
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)

            span = math.hypot(x2 - x1, y2 - y1) or 1.0
            if span > 153:
                Z_real = (-0.125 * span) + 44.125
            elif 107 < span <= 153:
                Z_real = (-0.217391304348 * span) + 58.2608695652
            else:
                Z_real = (A * (span ** 2)) + (B * span) + C

            scrnCenter_x = w // 2
            scrnCenter_y = h // 2
            X_virtual = -(cx - scrnCenter_x)
            Y_virtual = -(cy - scrnCenter_y)

            X_real = X_virtual * (6.3 / span)
            Y_real = Y_virtual * (6.3 / span)

            pan_deg = None
            tilt_deg = None
            if TurretAngles is not None:
                try:
                    ang = TurretAngles(X_real + OFFSET_X, Y_real + OFFSET_Y, Z_real + OFFSET_Z)
                    ang.getAngles()
                    pan_deg = ang.getTheta_x()
                    tilt_deg = ang.getTheta_y()
                except Exception as e:
                    self.get_logger().warning(f"TurretAngles failed: {e}")

            pt = Point()
            pt.x = float(X_real)
            pt.y = float(Y_real)
            pt.z = float(Z_real)
            self.pub_pt.publish(pt)

            txt = f"X:{int(X_real)} Y:{int(Y_real)} Z:{int(Z_real)}"
            if pan_deg is not None and tilt_deg is not None:
                txt += f" pan:{pan_deg:.1f} tilt:{tilt_deg:.1f}"
            cv2.putText(frame, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 255, 200), 2)
            cv2.putText(frame, f"Area:{int(area)} Span:{int(span)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
        else:
            cv2.putText(frame, "No detection", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150,150,150), 2)

        now = time.time()
        dt = max(1e-6, now - self.last_time)
        fps = 1.0 / dt
        self.last_time = now
        cv2.putText(frame, f"Frame:{self.frame_idx} FPS:{fps:.1f}", (10, STREAM_H-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220,220,220), 1)

        try:
            outmsg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            outmsg.header = msg.header
            self.pub_img.publish(outmsg)
        except Exception as e:
            self.get_logger().warning(f"Failed to publish annotated image: {e}")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
