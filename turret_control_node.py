import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
import time

# try import pigpio; if missing, run in simulated/no-op mode (useful for debugging on desktop)
try:
    import pigpio
    PIGPIO_AVAILABLE = True
except Exception:
    PIGPIO_AVAILABLE = False
    pigpio = None

# Defaults (same as your script)
GPIO_PAN = int(__import__('os').environ.get("GPIO_PAN", 17))
GPIO_TILT = int(__import__('os').environ.get("GPIO_TILT", 27))
GPIO_LASER = int(__import__('os').environ.get("GPIO_LASER", 25))
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
START_ANGLE = 90.0
RAMP_DEG_PER_SEC = 60.0
LOOP_HZ = 20.0

def pulse_from_angle(angle):
    angle = max(0.0, min(180.0, float(angle)))
    return int(SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US))

def ramp_val(current, target, max_step):
    if abs(target - current) <= max_step:
        return target
    return current + max_step if target > current else current - max_step

class TurretControlNode(Node):
    def __init__(self):
        super().__init__('turret_control_node')
        qos = rclpy.qos.QoSProfile(depth=10)

        # Subscribe to geometry_msgs/Point (x=pan_deg, y=tilt_deg)
        self.sub = self.create_subscription(Point, '/turret/angles', self.angles_cb, qos)
        self.pub_status = self.create_publisher(String, '/turret/status', qos)
        self.pub_laser = self.create_publisher(Bool, '/laser/state', qos)

        self.pi = None
        if PIGPIO_AVAILABLE:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                self.get_logger().fatal('pigpiod not running — start with: sudo systemctl start pigpiod')
                raise SystemExit('pigpiod not running')
            self.get_logger().info('pigpio connected')
            self.pi.set_mode(GPIO_LASER, pigpio.OUTPUT)
        else:
            self.get_logger().warning('pigpio not available: running in simulation (no hardware pulses)')

        # center on startup
        self.current_pan = START_ANGLE
        self.current_tilt = START_ANGLE
        self.target_pan = START_ANGLE
        self.target_tilt = START_ANGLE
        self.max_step = RAMP_DEG_PER_SEC / LOOP_HZ
        self.laser_on = False
        self.get_logger().info('Turret control initialized')

        # center briefly and stop pulses (if pigpio available)
        if self.pi:
            self.pi.set_servo_pulsewidth(GPIO_PAN, pulse_from_angle(START_ANGLE))
            self.pi.set_servo_pulsewidth(GPIO_TILT, pulse_from_angle(START_ANGLE))
            time.sleep(0.5)
            self.pi.set_servo_pulsewidth(GPIO_PAN, 0)
            self.pi.set_servo_pulsewidth(GPIO_TILT, 0)

        self.timer = self.create_timer(1.0/LOOP_HZ, self.timer_cb)

    def angles_cb(self, msg: Point):
        # accept geometry_msgs/Point as produced by TrackingNode
        try:
            self.target_pan = float(msg.x)
            self.target_tilt = float(msg.y)
        except Exception as e:
            self.get_logger().warning(f"angles_cb conversion error: {e}")

    def timer_cb(self):
        self.current_pan = ramp_val(self.current_pan, self.target_pan, self.max_step)
        self.current_tilt = ramp_val(self.current_tilt, self.target_tilt, self.max_step)

        pulse_pan = pulse_from_angle(self.current_pan)
        pulse_tilt = pulse_from_angle(self.current_tilt)

        if self.pi:
            try:
                self.pi.set_servo_pulsewidth(GPIO_PAN, pulse_pan)
                self.pi.set_servo_pulsewidth(GPIO_TILT, pulse_tilt)
            except Exception as e:
                self.get_logger().warning(f"pigpio write failed: {e}")
        else:
            # simulation: log instead of pulse
            self.get_logger().debug(f"(sim) would set pulses pan={pulse_pan} tilt={pulse_tilt}")

        st = String()
        st.data = f'pan:{self.current_pan:.2f},tilt:{self.current_tilt:.2f}'
        self.pub_status.publish(st)

        if abs(self.current_pan - START_ANGLE) > 0.5 or abs(self.current_tilt - START_ANGLE) > 0.5:
            if not self.laser_on:
                self.laser_on = True
                if self.pi:
                    self.pi.write(GPIO_LASER, 1)
                self.pub_laser.publish(Bool(data=True))
        else:
            if self.laser_on:
                self.laser_on = False
                if self.pi:
                    self.pi.write(GPIO_LASER, 0)
                self.pub_laser.publish(Bool(data=False))

    def destroy_node(self):
        try:
            if self.pi:
                self.pi.set_servo_pulsewidth(GPIO_PAN, 0)
                self.pi.set_servo_pulsewidth(GPIO_TILT, 0)
                self.pi.write(GPIO_LASER, 0)
                self.pi.stop()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TurretControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
