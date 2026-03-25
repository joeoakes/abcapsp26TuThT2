#!/usr/bin/env python3
"""
apriltag_detector.py
ROS2 node that subscribes to the camera feed, detects AprilTags,
and publishes detection results.

Usage:
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 apriltag_detector.py

Subscribes to: /camera/image_raw
Publishes to:  /apriltag/detections (String - JSON)
"""

import json
import math
import threading
import time

import numpy as np
from PIL import Image
from pupil_apriltags import Detector

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String


# Configuration
CAMERA_TOPIC = "/camera/image_raw"
DETECTION_TOPIC = "/apriltag/detections"
TAG_FAMILY = "tag36h11"
TAG_SIZE_M = 0.152  # Physical tag size in meters - adjust to your tags

# Camera intrinsics (adjust to your camera)
# These are approximate for the OV5647 at 640x480
CAMERA_FX = 460.0
CAMERA_FY = 460.0
CAMERA_CX = 320.0
CAMERA_CY = 240.0

# Detection rate limiting
DETECT_INTERVAL = 0.1  # Process at most 10 frames per second


class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__("apriltag_detector")

        # AprilTag detector
        self.detector = Detector(
            families=TAG_FAMILY,
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        self.get_logger().info(f"AprilTag detector initialized (family: {TAG_FAMILY})")

        # Publisher for detection results
        self.detection_pub = self.create_publisher(String, DETECTION_TOPIC, 10)

        # Subscriber for camera images
        self.camera_sub = self.create_subscription(
            RosImage,
            CAMERA_TOPIC,
            self._image_callback,
            10
        )
        self.get_logger().info(f"Subscribing to {CAMERA_TOPIC}")
        self.get_logger().info(f"Publishing to {DETECTION_TOPIC}")

        # Rate limiting
        self._last_detect_time = 0.0
        self._frame_count = 0
        self._detect_count = 0
        self._tag_count = 0

        # Stats timer
        self.create_timer(5.0, self._print_stats)

    def _image_callback(self, msg):
        self._frame_count += 1

        # Rate limit detection
        now = time.time()
        if now - self._last_detect_time < DETECT_INTERVAL:
            return
        self._last_detect_time = now

        try:
            # Convert ROS Image to grayscale numpy array
            gray = self._ros_image_to_gray(msg)
            if gray is None:
                return

            # Detect AprilTags
            tags = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY),
                tag_size=TAG_SIZE_M
            )

            self._detect_count += 1

            # Build detection results
            detections = []
            for tag in tags:
                # Extract pose
                tx, ty, tz = tag.pose_t.flatten().tolist()

                # Rotation matrix to roll/pitch/yaw
                R = tag.pose_R
                yaw = math.atan2(R[1, 0], R[0, 0])
                pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2))
                roll = math.atan2(R[2, 1], R[2, 2])

                # Corner positions (for visualization)
                corners = tag.corners.tolist()

                detection = {
                    "tag_id": tag.tag_id,
                    "family": TAG_FAMILY,
                    "confidence": round(tag.decision_margin, 2),
                    "position": {
                        "x": round(tx, 4),
                        "y": round(ty, 4),
                        "z": round(tz, 4)
                    },
                    "rotation": {
                        "roll": round(math.degrees(roll), 1),
                        "pitch": round(math.degrees(pitch), 1),
                        "yaw": round(math.degrees(yaw), 1)
                    },
                    "corners": corners
                }
                detections.append(detection)

            self._tag_count += len(detections)

            # Publish results
            result = {
                "timestamp": now,
                "frame": self._frame_count,
                "count": len(detections),
                "tags": detections
            }

            msg_out = String()
            msg_out.data = json.dumps(result)
            self.detection_pub.publish(msg_out)

            # Log when tags are found
            if detections:
                best = max(detections, key=lambda t: t["confidence"])
                self.get_logger().info(
                    f"Detected {len(detections)} tag(s) - "
                    f"Best: ID={best['tag_id']} "
                    f"pos=({best['position']['x']:.2f}, {best['position']['y']:.2f}, {best['position']['z']:.2f})m "
                    f"yaw={best['rotation']['yaw']:.1f}deg"
                )

        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")

    def _ros_image_to_gray(self, msg):
        """Convert a ROS Image message to a grayscale numpy array."""
        try:
            width = msg.width
            height = msg.height
            encoding = msg.encoding

            # Convert raw bytes to numpy array
            raw = np.frombuffer(msg.data, dtype=np.uint8)

            if encoding == "rgb8":
                img = raw.reshape((height, width, 3))
                # RGB to grayscale: 0.299*R + 0.587*G + 0.114*B
                gray = (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2]).astype(np.uint8)

            elif encoding == "bgr8":
                img = raw.reshape((height, width, 3))
                gray = (0.299 * img[:, :, 2] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 0]).astype(np.uint8)

            elif encoding == "mono8":
                gray = raw.reshape((height, width))

            elif encoding in ("rgba8", "bgra8"):
                img = raw.reshape((height, width, 4))
                if encoding == "rgba8":
                    gray = (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2]).astype(np.uint8)
                else:
                    gray = (0.299 * img[:, :, 2] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 0]).astype(np.uint8)

            else:
                self.get_logger().warn(f"Unsupported image encoding: {encoding}")
                return None

            return gray

        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return None

    def _print_stats(self):
        self.get_logger().info(
            f"Stats: {self._frame_count} frames received, "
            f"{self._detect_count} processed, "
            f"{self._tag_count} total tags detected"
        )


def main():
    rclpy.init()
    node = AprilTagDetectorNode()

    print(f"[APRILTAG] Detector running")
    print(f"[APRILTAG] Subscribing to {CAMERA_TOPIC}")
    print(f"[APRILTAG] Publishing to {DETECTION_TOPIC}")
    print(f"[APRILTAG] Tag family: {TAG_FAMILY}, size: {TAG_SIZE_M}m")
    print(f"[APRILTAG] Ctrl+C to stop")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[APRILTAG] Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[APRILTAG] Done.")


if __name__ == "__main__":
    main()
