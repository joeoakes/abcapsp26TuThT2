\# AprilTag Pose Camera (Python)



This application uses your computer camera to detect AprilTags and estimate:



\- Tag Family

\- Tag ID

\- Position: X, Y, Z (meters from camera)

\- Orientation: Roll, Pitch, Yaw



It works on:



\- Windows

\- macOS

\- Linux (Ubuntu recommended)



---



\# Requirements



\- Python 3.10, 3.11, or 3.12

\- A webcam

\- pip



Check Python version:



&nbsp;   python --version



or



&nbsp;   python3 --version



---



\# 1️⃣ Create a Virtual Environment (Recommended)



Windows:



&nbsp;   python -m venv venv

&nbsp;   venv\\Scripts\\activate



macOS / Linux:



&nbsp;   python3 -m venv venv

&nbsp;   source venv/bin/activate



---



\# 2️⃣ Install Required Packages



Install dependencies:



&nbsp;   pip install opencv-python numpy apriltag



If using Python 3.12 and apriltag fails to build:



&nbsp;   pip install opencv-python numpy pupil-apriltags



(Then modify import in script to use `from pupil\_apriltags import Detector`.)



---



\# 3️⃣ Camera Index Selection



If you have multiple cameras:



\- 0 = default camera

\- 1 = external webcam

\- 2 = additional devices



In the script, change:



&nbsp;   cv2.VideoCapture(0)



to:



&nbsp;   cv2.VideoCapture(1)



For example, if using Logitech BRIO.



---



\# 4️⃣ Running the Application



Windows:



&nbsp;   python apriltag\_pose\_cam.py



macOS / Linux:



&nbsp;   python3 apriltag\_pose\_cam.py



---



\# 5️⃣ Common Fixes



\## Camera Not Opening



Try changing camera index to 1 or 2.



On Linux, you can list devices:



&nbsp;   ls /dev/video\*



---



\## macOS Camera Permission



Go to:



System Settings → Privacy \& Security → Camera



Allow Terminal or Python.



---



\## Windows Camera Permission



Settings → Privacy → Camera  

Enable camera access for desktop apps.



---



\## If apriltag fails to install on Windows



Install Microsoft C++ Build Tools:



https://visualstudio.microsoft.com/visual-cpp-build-tools/



Then reinstall:



&nbsp;   pip install apriltag



---



\# 6️⃣ Printing AprilTags



You can generate printable tags here:



https://april.eecs.umich.edu/software/apriltag.html



Recommended family:



&nbsp;   tag36h11



Print at high contrast on white paper.



---



\# 7️⃣ Exit Program



Press:



&nbsp;   q



to quit.



---



\# Example Output



Tag: tag36h11

ID: 4

Position (m): X=0.12 Y=-0.05 Z=0.83

Orientation (deg): Roll=2.1 Pitch=-1.5 Yaw=15.3



---



\# Notes



\- For accurate pose estimation, you must set correct camera calibration parameters in the script.

\- Use a chessboard calibration with OpenCV for best results.



---



\# Author



Oakes

