#!/usr/bin/env python3
"""
overhead_tracker.py
Overhead camera AprilTag tracker for the maze space.

Detects AprilTags on the Mini-Pupper robot and maze reference points
from an overhead USB webcam. Displays a live view with tracking overlay
and optionally POSTs position data to the logging/AI servers.

Usage:
    source ~/abcapsp26TuThT2/apriltag/.venv/bin/activate
    python3 overhead_tracker.py

Controls:
    ESC - quit
    C   - toggle camera index (0, 1, 2)
    R   - reset maze origin calibration
    S   - save current frame as screenshot
"""

import cv2
import numpy as np
from pupil_apriltags import Detector
import math
import json
import time
import os
import sys

# ─── Configuration ────────────────────────────────────────

# Robot tag IDs (mounted on Mini-Pupper)
ROBOT_TAG_IDS = {288, 289, 290, 301}

# Maze reference tag IDs (placed at fixed points in maze space)
# Update these with the actual tag IDs you place in the maze
MAZE_REF_TAG_IDS = set()  # e.g. {0, 1, 2, 3}

# Maze origin tag ID - the tag that defines (0,0) in maze coordinates
# Set this to one of your maze reference tags once you have them
MAZE_ORIGIN_TAG_ID = None  # e.g. 0

# Tag family and size
TAG_FAMILY = "tag36h11"
TAG_SIZE_M = 0.152  # Physical tag size in meters

# Camera settings
CAMERA_INDEX = 0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# Camera intrinsics (approximate for a typical USB webcam at 1280x720)
# Adjust these for your specific webcam for better accuracy
CAMERA_FX = 900.0
CAMERA_FY = 900.0
CAMERA_CX = 640.0
CAMERA_CY = 360.0

# Display settings
WINDOW_NAME = "Overhead Maze Tracker"
FONT = cv2.FONT_HERSHEY_SIMPLEX

# Colors (BGR)
COLOR_ROBOT = (0, 255, 0)       # Green for robot tags
COLOR_MAZE_REF = (255, 165, 0)  # Orange for maze reference tags
COLOR_UNKNOWN = (0, 255, 255)   # Yellow for unknown tags
COLOR_TEXT = (255, 255, 255)     # White text
COLOR_SHADOW = (0, 0, 0)        # Black text shadow
COLOR_TRAIL = (0, 200, 200)     # Cyan for robot trail
COLOR_HUD_BG = (20, 20, 20)     # Dark background for HUD


# ─── Helper Functions ─────────────────────────────────────

def rotation_matrix_to_yaw(R):
    """Extract yaw angle from rotation matrix."""
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def draw_text_shadowed(frame, text, pos, scale=0.6, color=COLOR_TEXT, thickness=2):
    """Draw text with a shadow for readability."""
    x, y = pos
    cv2.putText(frame, text, (x + 1, y + 1), FONT, scale, COLOR_SHADOW, thickness + 1)
    cv2.putText(frame, text, (x, y), FONT, scale, color, thickness)


def draw_tag_outline(frame, corners, color, thickness=2):
    """Draw the tag outline and center point."""
    pts = corners.astype(int)
    for i in range(4):
        p0 = tuple(pts[i])
        p1 = tuple(pts[(i + 1) % 4])
        cv2.line(frame, p0, p1, color, thickness)

    # Draw center point
    cx = int(np.mean(corners[:, 0]))
    cy = int(np.mean(corners[:, 1]))
    cv2.circle(frame, (cx, cy), 5, color, -1)

    return (cx, cy)


def classify_tag(tag_id):
    """Classify a tag as robot, maze reference, or unknown."""
    if tag_id in ROBOT_TAG_IDS:
        return "robot", COLOR_ROBOT
    elif tag_id in MAZE_REF_TAG_IDS:
        return "maze_ref", COLOR_MAZE_REF
    else:
        return "unknown", COLOR_UNKNOWN


# ─── Tracker State ────────────────────────────────────────

class TrackerState:
    def __init__(self):
        self.robot_positions = []  # Trail of (x, y) pixel positions
        self.max_trail_length = 200
        self.robot_world_pos = None  # (x, y, z) in meters
        self.robot_yaw = None
        self.robot_visible = False
        self.maze_refs = {}  # tag_id -> {pos, yaw, corners}
        self.fps = 0.0
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.last_fps_count = 0
        self.screenshot_count = 0

    def update_fps(self):
        """Update FPS counter."""
        now = time.time()
        elapsed = now - self.last_fps_time
        if elapsed >= 1.0:
            self.fps = (self.frame_count - self.last_fps_count) / elapsed
            self.last_fps_time = now
            self.last_fps_count = self.frame_count

    def add_robot_position(self, px, py):
        """Add a pixel position to the robot trail."""
        self.robot_positions.append((px, py))
        if len(self.robot_positions) > self.max_trail_length:
            self.robot_positions.pop(0)

    def clear_trail(self):
        """Clear the robot trail."""
        self.robot_positions.clear()


# ─── Main ─────────────────────────────────────────────────

def main():
    cam_index = CAMERA_INDEX

    # Try to open camera
    print(f"[TRACKER] Opening camera index {cam_index}...")
    cap = cv2.VideoCapture(cam_index, cv2.CAP_AVFOUNDATION)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    if not cap.isOpened():
        print(f"[ERROR] Could not open camera {cam_index}. Try a different index.")
        print(f"[ERROR] Use 'C' key to cycle camera index while running.")
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[TRACKER] Camera opened: {actual_w}x{actual_h}")

    # Update camera intrinsics based on actual resolution
    fx = CAMERA_FX * (actual_w / CAMERA_WIDTH)
    fy = CAMERA_FY * (actual_h / CAMERA_HEIGHT)
    cx = actual_w / 2.0
    cy = actual_h / 2.0

    # Initialize detector
    detector = Detector(
        families=TAG_FAMILY,
        nthreads=2,
        quad_decimate=2.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )
    print(f"[TRACKER] AprilTag detector ready (family: {TAG_FAMILY})")
    print(f"[TRACKER] Robot tag IDs: {sorted(ROBOT_TAG_IDS)}")
    if MAZE_REF_TAG_IDS:
        print(f"[TRACKER] Maze reference tag IDs: {sorted(MAZE_REF_TAG_IDS)}")
    else:
        print(f"[TRACKER] No maze reference tags configured yet")
    print(f"[TRACKER] Controls: ESC=quit, C=change camera, R=reset trail, S=screenshot")
    print()

    state = TrackerState()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[ERROR] Failed to read frame")
            break

        state.frame_count += 1
        state.update_fps()

        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect tags
        tags = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(fx, fy, cx, cy),
            tag_size=TAG_SIZE_M
        )

        # Process detections
        state.robot_visible = False
        best_robot_tag = None
        best_robot_margin = 0

        for tag in tags:
            tag_type, color = classify_tag(tag.tag_id)

            # Draw tag outline
            center_px = draw_tag_outline(frame, tag.corners, color, 2)

            # Extract pose
            tx, ty, tz = tag.pose_t.flatten().tolist()
            yaw = rotation_matrix_to_yaw(tag.pose_R)

            # Draw tag info
            label = f"ID:{tag.tag_id} ({tag_type})"
            draw_text_shadowed(frame, label, (center_px[0] + 10, center_px[1] - 20),
                               scale=0.5, color=color)

            pos_label = f"z:{tz:.2f}m yaw:{yaw:.0f}deg"
            draw_text_shadowed(frame, pos_label, (center_px[0] + 10, center_px[1]),
                               scale=0.4, color=color)

            if tag_type == "robot":
                # Track the best (highest confidence) robot tag
                if tag.decision_margin > best_robot_margin:
                    best_robot_margin = tag.decision_margin
                    best_robot_tag = tag
                    state.robot_visible = True
                    state.robot_world_pos = (tx, ty, tz)
                    state.robot_yaw = yaw

            elif tag_type == "maze_ref":
                state.maze_refs[tag.tag_id] = {
                    "pos": (tx, ty, tz),
                    "yaw": yaw,
                    "center_px": center_px
                }

        # Update robot trail
        if best_robot_tag is not None:
            cx_robot = int(np.mean(best_robot_tag.corners[:, 0]))
            cy_robot = int(np.mean(best_robot_tag.corners[:, 1]))
            state.add_robot_position(cx_robot, cy_robot)

            # Draw direction arrow from robot center
            arrow_len = 40
            angle_rad = math.radians(state.robot_yaw)
            arrow_end = (
                int(cx_robot + arrow_len * math.cos(angle_rad)),
                int(cy_robot + arrow_len * math.sin(angle_rad))
            )
            cv2.arrowedLine(frame, (cx_robot, cy_robot), arrow_end,
                            COLOR_ROBOT, 3, tipLength=0.3)

        # Draw robot trail
        if len(state.robot_positions) > 1:
            for i in range(1, len(state.robot_positions)):
                alpha = i / len(state.robot_positions)
                thickness = max(1, int(alpha * 3))
                color_fade = (
                    int(COLOR_TRAIL[0] * alpha),
                    int(COLOR_TRAIL[1] * alpha),
                    int(COLOR_TRAIL[2] * alpha)
                )
                cv2.line(frame, state.robot_positions[i - 1],
                         state.robot_positions[i], color_fade, thickness)

        # ─── HUD Overlay ─────────────────────────────────

        # Top-left: status panel
        hud_h = 120
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (320, hud_h), COLOR_HUD_BG, -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

        draw_text_shadowed(frame, "OVERHEAD MAZE TRACKER", (10, 22),
                           scale=0.5, color=(0, 200, 255))

        draw_text_shadowed(frame, f"FPS: {state.fps:.1f}", (10, 45),
                           scale=0.4, color=COLOR_TEXT)

        draw_text_shadowed(frame, f"Tags detected: {len(tags)}", (10, 65),
                           scale=0.4, color=COLOR_TEXT)

        # Robot status
        if state.robot_visible:
            robot_status = "ROBOT: VISIBLE"
            robot_color = COLOR_ROBOT
        else:
            robot_status = "ROBOT: NOT VISIBLE"
            robot_color = (0, 0, 255)

        draw_text_shadowed(frame, robot_status, (10, 85), scale=0.4, color=robot_color)

        if state.robot_world_pos:
            x, y, z = state.robot_world_pos
            draw_text_shadowed(frame, f"Pos: x={x:.2f} y={y:.2f} z={z:.2f}m",
                               (10, 105), scale=0.4, color=COLOR_TEXT)

        # Bottom: legend
        legend_y = actual_h - 30
        draw_text_shadowed(frame, "GREEN=Robot", (10, legend_y),
                           scale=0.4, color=COLOR_ROBOT)
        draw_text_shadowed(frame, "ORANGE=Maze Ref", (160, legend_y),
                           scale=0.4, color=COLOR_MAZE_REF)
        draw_text_shadowed(frame, "YELLOW=Unknown", (350, legend_y),
                           scale=0.4, color=COLOR_UNKNOWN)

        # Show frame
        cv2.imshow(WINDOW_NAME, frame)

        # Handle key input
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            print("[TRACKER] Exiting...")
            break

        elif key == ord('c') or key == ord('C'):
            # Cycle camera index
            cap.release()
            cam_index = (cam_index + 1) % 3
            print(f"[TRACKER] Switching to camera {cam_index}...")
            cap = cv2.VideoCapture(cam_index)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            if not cap.isOpened():
                print(f"[ERROR] Camera {cam_index} not available")

        elif key == ord('r') or key == ord('R'):
            state.clear_trail()
            print("[TRACKER] Trail cleared")

        elif key == ord('s') or key == ord('S'):
            filename = f"overhead_screenshot_{state.screenshot_count:03d}.jpg"
            cv2.imwrite(filename, frame)
            print(f"[TRACKER] Screenshot saved: {filename}")
            state.screenshot_count += 1

    cap.release()
    cv2.destroyAllWindows()
    print("[TRACKER] Done.")


if __name__ == "__main__":
    main()
