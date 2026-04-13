#!/usr/bin/env python3
"""
ros2_https_controller.py
HTTPS server + ROS2 node for Mini-Pupper control and camera streaming.

Endpoints:
    POST /move          - Send movement commands {"move_dir": "forward/backward/left/right/stop"}
    GET  /camera        - Get latest JPEG frame from camera
    GET  /tags          - Get latest AprilTag detections (JSON)
    GET  /health        - Health check

Usage:
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 ros2_https_controller.py

mTLS: Expects per-device certs at certs/pupper-1.crt, certs/pupper-1.key, certs/ca.crt.
Clients (Game HAT) must present a cert signed by ca.crt to connect.
"""

import json
import math
import os
import ssl
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# Configuration
HTTPS_PORT = int(os.environ.get("LISTEN_PORT", "8444"))
CERT_FILE  = os.environ.get("TLS_CERT", "certs/pupper-1.crt")
KEY_FILE   = os.environ.get("TLS_KEY",  "certs/pupper-1.key")
CA_FILE    = os.environ.get("TLS_CA",   "certs/ca.crt")

DEFAULT_LINEAR_SPEED = 0.15
DEFAULT_ANGULAR_SPEED = 0.5
SAFETY_TIMEOUT = 5.0
CMD_VEL_TOPIC = "/cmd_vel"
CAMERA_TOPIC = "/camera/image_raw/compressed"
APRILTAG_TOPIC = "/apriltag/detections"
LOCALIZATION_TOPIC = "/apriltag/localization"
ODOM_TOPIC = "/odom"
PUBLISH_RATE = 10.0


class PupperController(Node):
    def __init__(self):
        super().__init__("pupper_https_controller")

        # Movement publisher
        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.get_logger().info(f"Publishing to {CMD_VEL_TOPIC}")

        # Camera subscriber
        self._camera_lock = threading.Lock()
        self._latest_frame = None
        self._frame_count = 0
        self.camera_sub = self.create_subscription(
            CompressedImage,
            CAMERA_TOPIC,
            self._camera_callback,
            10
        )
        self.get_logger().info(f"Subscribing to {CAMERA_TOPIC}")

        # AprilTag subscriber
        self._tag_lock = threading.Lock()
        self._latest_tags = '{"count":0,"tags":[]}'
        self.tag_sub = self.create_subscription(
            String,
            APRILTAG_TOPIC,
            self._tag_callback,
            10
        )
        self.get_logger().info(f"Subscribing to {APRILTAG_TOPIC}")

        # Odometry subscriber
        self._odom_lock = threading.Lock()
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_valid = False
        self.odom_sub = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self._odom_callback,
            10
        )
        self.get_logger().info(f"Subscribing to {ODOM_TOPIC}")

        # Localization subscriber (from apriltag detector seeing reference cube)
        self._loc_lock = threading.Lock()
        self._latest_loc = '{"valid":false}'
        self.loc_sub = self.create_subscription(
            String,
            LOCALIZATION_TOPIC,
            self._loc_callback,
            10
        )
        self.get_logger().info(f"Subscribing to {LOCALIZATION_TOPIC}")

        # Velocity state
        self._vel_lock = threading.Lock()
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._last_command_time = time.time()

        # Publish velocity at steady rate
        self._timer = self.create_timer(1.0 / PUBLISH_RATE, self._publish_velocity)

    def _camera_callback(self, msg):
        with self._camera_lock:
            self._latest_frame = bytes(msg.data)
            self._frame_count += 1

    def _tag_callback(self, msg):
        with self._tag_lock:
            self._latest_tags = msg.data

    def _odom_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert quaternion to yaw (heading in degrees)
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # Yaw from quaternion (only z rotation for ground robot)
        yaw_rad = 2.0 * math.atan2(qz, qw)
        yaw_deg = math.degrees(yaw_rad)

        with self._odom_lock:
            self._odom_x = x
            self._odom_y = y
            self._odom_yaw = yaw_deg
            self._odom_valid = True

    def get_latest_frame(self):
        with self._camera_lock:
            return self._latest_frame

    def get_frame_count(self):
        with self._camera_lock:
            return self._frame_count

    def get_latest_tags(self):
        with self._tag_lock:
            return self._latest_tags

    def get_odom(self):
        with self._odom_lock:
            return {
                "valid": self._odom_valid,
                "x": round(self._odom_x, 4),
                "y": round(self._odom_y, 4),
                "yaw": round(self._odom_yaw, 1)
            }

    def _loc_callback(self, msg):
        with self._loc_lock:
            self._latest_loc = msg.data

    def get_latest_localization(self):
        with self._loc_lock:
            return self._latest_loc

    def set_velocity(self, linear_x, angular_z):
        with self._vel_lock:
            self._linear_x = linear_x
            self._angular_z = angular_z
            self._last_command_time = time.time()

    def stop(self):
        with self._vel_lock:
            self._linear_x = 0.0
            self._angular_z = 0.0

    def _publish_velocity(self):
        with self._vel_lock:
            elapsed = time.time() - self._last_command_time
            if elapsed > SAFETY_TIMEOUT and (self._linear_x != 0.0 or self._angular_z != 0.0):
                self.get_logger().warn("Safety timeout - stopping robot")
                self._linear_x = 0.0
                self._angular_z = 0.0

            msg = Twist()
            msg.linear.x = self._linear_x
            msg.angular.z = self._angular_z

        self.publisher.publish(msg)


# Global reference set in main()
_ros_node = None


class MoveHandler(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        msg = format % args
        if "/camera" not in msg and "/odom" not in msg and "/localization" not in msg:
            print(f"[HTTPS] {self.address_string()} - {msg}")

    def _send_json(self, code, body):
        data = json.dumps(body).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(data))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(data)

    def _send_jpeg(self, jpeg_data):
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", len(jpeg_data))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache, no-store")
        self.end_headers()
        self.wfile.write(jpeg_data)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        if self.path == "/camera":
            frame = _ros_node.get_latest_frame()
            if frame:
                self._send_jpeg(frame)
            else:
                self._send_json(503, {"error": "No camera frame available yet"})

        elif self.path == "/tags":
            tags_json = _ros_node.get_latest_tags()
            data = tags_json.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", len(data))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            self.wfile.write(data)

        elif self.path == "/odom":
            odom_data = _ros_node.get_odom()
            self._send_json(200, odom_data)

        elif self.path == "/localization":
            loc_json = _ros_node.get_latest_localization()
            data = loc_json.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", len(data))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            self.wfile.write(data)

        elif self.path == "/health":
            self._send_json(200, {
                "ok": True,
                "node": "pupper_https_controller",
                "camera_frames": _ros_node.get_frame_count()
            })

        else:
            self._send_json(200, {
                "service": "Mini-Pupper HTTPS ROS2 Controller",
                "endpoints": {
                    "POST /move": "Movement commands",
                    "GET /camera": "Latest JPEG frame",
                    "GET /tags": "Latest AprilTag detections",
                    "GET /odom": "Robot odometry (position + heading)",
                    "GET /localization": "Robot position from reference cube",
                    "GET /health": "Health check"
                }
            })

    def do_POST(self):
        if self.path != "/move":
            self._send_json(404, {"error": "Not found. POST to /move"})
            return

        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            self._send_json(400, {"error": "Empty request body"})
            return

        try:
            body = self.rfile.read(length).decode("utf-8")
            data = json.loads(body)
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            self._send_json(400, {"error": f"Invalid JSON: {e}"})
            return

        move_dir = data.get("move_dir", "").lower().strip()
        if not move_dir:
            self._send_json(400, {"error": "Missing 'move_dir' field"})
            return

        linear_speed = float(data.get("linear_speed", DEFAULT_LINEAR_SPEED))
        angular_speed = float(data.get("angular_speed", DEFAULT_ANGULAR_SPEED))

        linear_x = 0.0
        angular_z = 0.0

        if move_dir == "forward":
            linear_x = linear_speed
        elif move_dir == "backward":
            linear_x = -linear_speed
        elif move_dir == "left":
            angular_z = angular_speed
        elif move_dir == "right":
            angular_z = -angular_speed
        elif move_dir == "stop":
            pass
        else:
            self._send_json(400, {"error": f"Unknown move_dir: '{move_dir}'"})
            return

        _ros_node.set_velocity(linear_x, angular_z)
        self._send_json(200, {
            "ok": True,
            "move_dir": move_dir,
            "linear_x": linear_x,
            "angular_z": angular_z
        })


class ThreadedHTTPServer(HTTPServer):
    """Handle each request in a separate thread for concurrent camera + move."""
    def process_request(self, request, client_address):
        t = threading.Thread(target=self._handle_request, args=(request, client_address))
        t.daemon = True
        t.start()

    def _handle_request(self, request, client_address):
        try:
            self.finish_request(request, client_address)
        except Exception:
            self.handle_error(request, client_address)
        finally:
            self.shutdown_request(request)


def run_https_server(port, certfile, keyfile, cafile):
    server = ThreadedHTTPServer(("0.0.0.0", port), MoveHandler)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=certfile, keyfile=keyfile)  # pupper-1 identity
    ctx.verify_mode = ssl.CERT_REQUIRED                      # require client cert
    ctx.load_verify_locations(cafile=cafile)                 # trust only our CA

    server.socket = ctx.wrap_socket(server.socket, server_side=True)

    print(f"[HTTPS] Listening on https://0.0.0.0:{port}")
    print(f"[HTTPS] mTLS enabled — clients must present a cert signed by {cafile}")
    print(f"[HTTPS] POST /move         - Movement commands")
    print(f"[HTTPS] GET  /camera        - Latest JPEG frame")
    print(f"[HTTPS] GET  /tags          - AprilTag detections")
    print(f"[HTTPS] GET  /odom          - Robot odometry")
    print(f"[HTTPS] GET  /localization  - Position from reference cube")
    print(f"[HTTPS] GET  /health        - Health check")
    server.serve_forever()


def main():
    global _ros_node

    for label, path in [("TLS cert", CERT_FILE), ("TLS key", KEY_FILE), ("CA cert", CA_FILE)]:
        if not os.path.exists(path):
            print(f"[ERROR] {label} not found: {path}")
            print(f"[ERROR] Run generate_certs.sh and SCP certs to this device.")
            return

    rclpy.init()
    _ros_node = PupperController()

    https_thread = threading.Thread(
        target=run_https_server,
        args=(HTTPS_PORT, CERT_FILE, KEY_FILE, CA_FILE),
        daemon=True
    )
    https_thread.start()

    print(f"[ROS2] Node 'pupper_https_controller' running")
    print(f"[ROS2] Publishing to {CMD_VEL_TOPIC} at {PUBLISH_RATE} Hz")
    print(f"[ROS2] Subscribing to {CAMERA_TOPIC}")
    print(f"[ROS2] Subscribing to {APRILTAG_TOPIC}")
    print(f"[ROS2] Subscribing to {LOCALIZATION_TOPIC}")
    print(f"[ROS2] Subscribing to {ODOM_TOPIC}")
    print(f"[ROS2] Safety timeout: {SAFETY_TIMEOUT}s")
    print(f"[INFO] Ready for commands. Ctrl+C to stop.")

    try:
        rclpy.spin(_ros_node)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
    finally:
        _ros_node.stop()
        msg = Twist()
        _ros_node.publisher.publish(msg)
        time.sleep(0.1)
        _ros_node.destroy_node()
        rclpy.shutdown()
        print("[INFO] Done.")


if __name__ == "__main__":
    main()
