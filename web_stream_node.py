import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Flask, Response, render_template_string
from threading import Thread
import time
import cv2

INDEX_HTML = """
<!doctype html>
<title>Arjuna Pro Turret (ROS)</title>
<style>
body { background:#111; color:#eee; font-family:Arial, sans-serif; margin:0; }
.container { max-width:1280px; margin:10px auto; text-align:center; }
img { width:90%; max-width:960px; border-radius:8px; border:2px solid #222; }
h1 { font-size:20px; margin:8px; }
.info { font-size:13px; color:#bbb; margin-bottom:6px; }
</style>
<div class="container">
  <h1>Arjuna — Professional Turret Stream (ROS)</h1>
  <div class="info">Move your hand. Press Ctrl+C to stop.</div>
  <img src="{{ url_for('video_feed') }}" alt="stream">
</div>
"""

FLASK_HOST = "0.0.0.0"
FLASK_PORT = 5000

app = Flask(__name__)
frame_buffer = None

class WebNode(Node):
    def __init__(self):
        super().__init__('web_stream_node')
        qos = rclpy.qos.QoSProfile(depth=2)
        self.sub = self.create_subscription(Image, '/camera/image_annotated', self.cb, qos)
        self.bridge = CvBridge()
        self.get_logger().info('Web stream node started')
        self.flask_thread = Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()

    def cb(self, msg: Image):
        global frame_buffer
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_buffer = frame
        except Exception as e:
            self.get_logger().warning(f'Failed to convert annotated image: {e}')

    def run_flask(self):
        @app.route('/')
        def index():
            return render_template_string(INDEX_HTML)

        @app.route('/stream')
        def video_feed():
            def gen():
                global frame_buffer
                while rclpy.ok():
                    frame = frame_buffer
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                    if not ret:
                        continue
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

        app.run(host=FLASK_HOST, port=FLASK_PORT, threaded=True, use_reloader=False)

def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()