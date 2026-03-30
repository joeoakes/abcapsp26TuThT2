#!/usr/bin/env python3
"""
apriltag_detector.py
ROS2 node that subscribes to the camera feed, detects AprilTags,
and publishes detection results + localization from reference cube.

Subscribes to: /camera/image_raw
Publishes to:  /apriltag/detections (String - JSON)
               /apriltag/localization (String - JSON) - robot position relative to reference cube

Usage:
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 apriltag_detector.py
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
LOCALIZATION_TOPIC = "/apriltag/localization"
TAG_FAMILY = "tag36h11"
TAG_SIZE_M = 0.0508  # Physical tag size in meters (~2 inches)

# Camera intrinsics (approximate for the OV5647 at 640x480)
CAMERA_FX = 460.0
CAMERA_FY = 460.0
CAMERA_CX = 320.0
CAMERA_CY = 240.0

# Detection rate limiting
DETECT_INTERVAL = 0.1  # Process at most 10 frames per second

# Reference cube configuration
# Tags on the cube at the center of the maze space
# The cube is at grid position (10, 7) in a 21x15 maze
REF_CUBE_TAG_IDS = {0, 1, 2, 3}
REF_CUBE_YAW_OFFSET = {
    0: 0.0,     # North face - no correction
    1: 90.0,    # East face - camera sees it from the west
    2: 180.0,   # South face - camera sees it from the north
    3: -90.0,   # West face - camera sees it from the east
}

# Robot tag IDs (to distinguish from reference tags)
ROBOT_TAG_IDS = {288, 289, 290, 301}


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

        # Publisher for localization data
        self.localization_pub = self.create_publisher(String, LOCALIZATION_TOPIC, 10)

        # Subscriber for camera images
        self.camera_sub = self.create_subscription(
            RosImage,
            CAMERA_TOPIC,
            self._image_callback,
            10
        )
        self.get_logger().info(f"Subscribing to {CAMERA_TOPIC}")
        self.get_logger().info(f"Publishing to {DETECTION_TOPIC}")
        self.get_logger().info(f"Publishing to {LOCALIZATION_TOPIC}")
        self.get_logger().info(f"Reference cube tags: {sorted(REF_CUBE_TAG_IDS)}")

        # Rate limiting
        self._last_detect_time = 0.0
        self._frame_count = 0
        self._detect_count = 0
        self._tag_count = 0
        self._ref_cube_sightings = 0

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

            # Build detection results and check for reference cube
            detections = []
            best_ref_tag = None
            best_ref_margin = 0

            for tag in tags:
                try:
                    # Skip if pose estimation failed
                    if tag.pose_t is None or tag.pose_R is None:
                        continue

                    # Extract pose
                    tx, ty, tz = tag.pose_t.flatten().tolist()

                    # Sanity check
                    if abs(tx) > 50 or abs(ty) > 50 or abs(tz) > 50 or tz <= 0:
                        continue

                    # Rotation matrix to roll/pitch/yaw
                    R = tag.pose_R
                    yaw = math.atan2(R[1, 0], R[0, 0])
                    pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2))
                    roll = math.atan2(R[2, 1], R[2, 2])

                    # Corner positions
                    corners = tag.corners.tolist()

                    # Classify tag
                    tag_type = "unknown"
                    if tag.tag_id in REF_CUBE_TAG_IDS:
                        tag_type = "reference"
                    elif tag.tag_id in ROBOT_TAG_IDS:
                        tag_type = "robot"

                    detection = {
                        "tag_id": tag.tag_id,
                        "tag_type": tag_type,
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

                    # Track best reference cube tag
                    if tag.tag_id in REF_CUBE_TAG_IDS:
                        if tag.decision_margin > best_ref_margin:
                            best_ref_margin = tag.decision_margin
                            best_ref_tag = tag

                except Exception:
                    continue

            self._tag_count += len(detections)

            # Publish all detections
            result = {
                "timestamp": now,
                "frame": self._frame_count,
                "count": len(detections),
                "tags": detections
            }

            msg_out = String()
            msg_out.data = json.dumps(result)
            self.detection_pub.publish(msg_out)

            # Process reference cube for localization
            if best_ref_tag is not None:
                self._publish_localization(best_ref_tag, now)

            # Log when tags are found
            if detections:
                best = max(detections, key=lambda t: t["confidence"])
                self.get_logger().info(
                    f"Detected {len(detections)} tag(s) - "
                    f"Best: ID={best['tag_id']} ({best['tag_type']}) "
                    f"pos=({best['position']['x']:.2f}, {best['position']['y']:.2f}, {best['position']['z']:.2f})m "
                    f"yaw={best['rotation']['yaw']:.1f}deg"
                )

        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")

    def _publish_localization(self, ref_tag, timestamp):
        """Calculate robot position relative to the reference cube and publish."""
        try:
            self._ref_cube_sightings += 1

            tag_id = ref_tag.tag_id
            tx, ty, tz = ref_tag.pose_t.flatten().tolist()
            R = ref_tag.pose_R
            raw_yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))

            # The pose gives us the tag's position relative to the camera.
            # Since the camera is on the robot, this tells us where the
            # reference cube is relative to the robot.
            # We invert this to get the robot's position relative to the cube.

            # Apply yaw offset based on which face of the cube we're seeing
            yaw_offset = REF_CUBE_YAW_OFFSET.get(tag_id, 0.0)

            # The tag's x, y, z in camera frame:
            #   x = right, y = down, z = forward (away from camera)
            # Distance to the tag
            distance = math.sqrt(tx**2 + ty**2 + tz**2)

            # Bearing to the tag in camera frame (horizontal angle)
            bearing = math.degrees(math.atan2(tx, tz))

            # The robot's heading relative to the cube face
            # raw_yaw is the tag's rotation as seen by the camera
            # yaw_offset corrects for which face we're seeing
            robot_heading_from_cube = raw_yaw + yaw_offset
            while robot_heading_from_cube > 180: robot_heading_from_cube -= 360
            while robot_heading_from_cube < -180: robot_heading_from_cube += 360

            # Robot position relative to reference cube (in meters)
            # Negative because the robot is opposite direction from the tag
            # tx = how far right the tag is from camera center
            # tz = how far forward the tag is from camera
            robot_rel_x = -tx  # If tag is to the right, robot is to the left of cube
            robot_rel_z = -tz  # Robot is behind where it's looking at the cube

            localization = {
                "timestamp": timestamp,
                "valid": True,
                "ref_tag_id": tag_id,
                "ref_tag_face": {0: "north", 1: "east", 2: "south", 3: "west"}.get(tag_id, "unknown"),
                "confidence": round(ref_tag.decision_margin, 2),
                "distance_to_cube": round(distance, 4),
                "bearing_to_cube": round(bearing, 1),
                "robot_relative_position": {
                    "x": round(robot_rel_x, 4),
                    "y": round(-ty, 4),
                    "z": round(robot_rel_z, 4)
                },
                "robot_heading_from_cube": round(robot_heading_from_cube, 1)
            }

            msg_out = String()
            msg_out.data = json.dumps(localization)
            self.localization_pub.publish(msg_out)

            self.get_logger().info(
                f"[LOC] Ref cube face={localization['ref_tag_face']} "
                f"dist={distance:.2f}m bearing={bearing:.1f}deg "
                f"heading={robot_heading_from_cube:.1f}deg"
            )

        except Exception as e:
            self.get_logger().error(f"Localization error: {e}")

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
            f"{self._tag_count} total tags detected, "
            f"{self._ref_cube_sightings} ref cube sightings"
        )


def main():
    rclpy.init()
    node = AprilTagDetectorNode()

    print(f"[APRILTAG] Detector running")
    print(f"[APRILTAG] Subscribing to {CAMERA_TOPIC}")
    print(f"[APRILTAG] Publishing to {DETECTION_TOPIC}")
    print(f"[APRILTAG] Publishing to {LOCALIZATION_TOPIC}")
    print(f"[APRILTAG] Tag family: {TAG_FAMILY}, size: {TAG_SIZE_M}m")
    print(f"[APRILTAG] Reference cube tags: {sorted(REF_CUBE_TAG_IDS)}")
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
