import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

# attempt robust import of the helper class `turret` from Angles.py
try:
    from .Angles import turret as TurretAngles
except Exception:
    try:
        from auto_turret.Angles import turret as TurretAngles
    except Exception as e:
        raise ImportError(f"Cannot import Angles.turret (tried relative and package-qualified): {e}")

# Tunables
OFFSET_X_CM = 12.0
OFFSET_Y_CM = 0.0
OFFSET_Z_CM = 7.0
PAN_CORRECTION_DEG = 0.0
TILT_CORRECTION_DEG = 0.0
SMOOTH_ALPHA = 0.25
LOOP_HZ = 20.0

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        qos = rclpy.qos.QoSProfile(depth=10)
        self.sub = self.create_subscription(Point, '/target/coords', self.cb_target, qos)
        self.pub_angles = self.create_publisher(Point, '/turret/angles', qos)
        self.pub_tracking = self.create_publisher(Bool, '/turret/status', qos)
        self.get_logger().info('Tracking node started')

        self.last_pan = 90.0
        self.last_tilt = 90.0
        self.has_target = False
        self.target_pt = Point()
        self._last_cmd_time = time.time()

        self.timer = self.create_timer(1.0/LOOP_HZ, self._control_loop)

    def cb_target(self, msg: Point):
        self.target_pt = msg
        self.has_target = True
        self._last_cmd_time = time.time()

    def _control_loop(self):
        now = time.time()
        if not self.has_target or (now - self._last_cmd_time) > 1.0:
            self.last_pan = self._smooth(self.last_pan, 90.0)
            self.last_tilt = self._smooth(self.last_tilt, 90.0)
            self._publish_angles(self.last_pan, self.last_tilt, tracking=False)
            if (now - self._last_cmd_time) > 2.0:
                self.has_target = False
            return

        X_real = float(self.target_pt.x)
        Y_real = float(self.target_pt.y)
        Z_real = float(self.target_pt.z)

        try:
            ang = TurretAngles(X_real, Y_real, Z_real)
            ang.offsets(OFFSET_X_CM, OFFSET_Y_CM, OFFSET_Z_CM, pan_correction=PAN_CORRECTION_DEG, tilt_correction=TILT_CORRECTION_DEG)
            pan_deg, tilt_deg = ang.getAngles()
        except Exception as e:
            self.get_logger().error(f"Angle computation failed: {e}")
            return

        sm_pan = self._smooth(self.last_pan, pan_deg)
        sm_tilt = self._smooth(self.last_tilt, tilt_deg)

        sm_pan = max(0.0, min(180.0, sm_pan))
        sm_tilt = max(0.0, min(180.0, sm_tilt))

        self._publish_angles(sm_pan, sm_tilt, tracking=True)
        self.last_pan, self.last_tilt = sm_pan, sm_tilt

    def _smooth(self, last, target):
        return (SMOOTH_ALPHA * target) + ((1.0 - SMOOTH_ALPHA) * last)

    def _publish_angles(self, pan_deg, tilt_deg, tracking=False):
        msg = Point()
        msg.x = float(pan_deg)
        msg.y = float(tilt_deg)
        msg.z = 0.0
        self.pub_angles.publish(msg)
        self.pub_tracking.publish(Bool(data=bool(tracking)))
        self.get_logger().info(f"Publish angles pan={pan_deg:.2f} tilt={tilt_deg:.2f} tracking={tracking}")

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()