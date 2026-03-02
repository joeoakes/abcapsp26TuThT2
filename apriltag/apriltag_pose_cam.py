import cv2
import numpy as np
from pupil_apriltags import Detector
import math

def rvec_to_rpy_degrees(rvec):
    R, _ = cv2.Rodrigues(rvec)
    # ZYX yaw-pitch-roll, R = Rz*Ry*Rx
    yaw = math.atan2(R[1,0], R[0,0])
    pitch = math.atan2(-R[2,0], math.sqrt(R[2,1]**2 + R[2,2]**2))
    roll = math.atan2(R[2,1], R[2,2])
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

def main():
    # ---- Camera selection ----
    # Try 0, 1, 2 if needed. For BRIO, 0 is often correct.
    cam_index = 0

    # Prefer DirectShow on Windows for better stability:
    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        raise RuntimeError("Could not open camera. Try cam_index=1 or 2.")

    # ---- AprilTag detector ----
    # families can be: "tag36h11", "tag25h9", "tag16h5", ...
    detector = Detector(
        families="tag36h11",
        nthreads=2,
        quad_decimate=2.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    # ---- Pose parameters ----
    tag_size_m = 0.152  # <-- set to the physical printed size (meters)

    # Camera intrinsics (fx, fy, cx, cy). These matter for Z distance.
    # If you don't know them, start with approximations:
    w, h = 1280, 720
    fx = fy = 900.0
    cx = w / 2.0
    cy = h / 2.0

    print("Press ESC to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detect() returns tags with pose if you pass camera_params + tag_size
        tags = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(fx, fy, cx, cy),
            tag_size=tag_size_m
        )

        # Pick the best tag (highest decision_margin)
        best = None
        if tags:
            best = max(tags, key=lambda t: t.decision_margin)

        if best is not None:
            # Type/family you configured in Detector.families
            tag_type = "tag36h11"
            tag_id = best.tag_id

            # Translation is in meters (camera frame)
            # +x right, +y down, +z forward (away from camera)
            tx, ty, tz = best.pose_t.flatten().tolist()

            # Rotation vector -> roll/pitch/yaw
            rvec = best.pose_R
            # pupil_apriltags returns pose_R as a 3x3 rotation matrix
            R = best.pose_R
            yaw = math.atan2(R[1,0], R[0,0])
            pitch = math.atan2(-R[2,0], math.sqrt(R[2,1]**2 + R[2,2]**2))
            roll = math.atan2(R[2,1], R[2,2])
            roll, pitch, yaw = map(math.degrees, (roll, pitch, yaw))

            msg1 = f"Type: {tag_type}  ID: {tag_id}"
            msg2 = f"X: {tx:.3f} m  Y: {ty:.3f} m  Z: {tz:.3f} m"
            msg3 = f"roll: {roll:.1f}  pitch: {pitch:.1f}  yaw: {yaw:.1f}"

            cv2.putText(frame, msg1, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
            cv2.putText(frame, msg2, (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
            cv2.putText(frame, msg3, (20,110), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)

            # draw outline
            corners = best.corners.astype(int)
            for i in range(4):
                p0 = tuple(corners[i])
                p1 = tuple(corners[(i+1) % 4])
                cv2.line(frame, p0, p1, (0,255,0), 2)

        cv2.imshow("AprilTag Pose (Python)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()