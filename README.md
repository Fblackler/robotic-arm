# Robotic Arm
A 3D printed 6-axis robotic arm with inverse kinematics, a web control panel, Xbox controller support, and camera-based hand tracking.

<!-- ![Demo](docs/demo.gif) -->

## Features

- **Inverse kinematics:** move the gripper to an X/Y/Z position with a set pitch, yaw and roll
- **Web control panel:** position and orientation sliders, gripper control, and live servo temperature, voltage and load graphs
- **Xbox controller support:** real-time control with the sticks and triggers
- **Hand tracking:** a webcam on the arm detects a hand and moves the arm to keep it centred in view
- **Synchronised motion:** all joints start and finish each move together
- **3D simulator:** preview arm movement without hardware

## How it works

All control goes through a single Flask server (`full_servo_hud.py`), the only component that talks to the hardware. Each interface sends it target positions over HTTP.

**Inverse kinematics.** The wrist position is found by working back from the target and the gripper direction. Shoulder and elbow angles are then searched for the closest match, and the forearm, wrist and hand joints set the final direction and roll. Targets outside any servo's limits are rejected.

**Hand tracking.** Camera frames are processed with MediaPipe's hand landmark model. The palm centre is compared with the image centre:

- Outside a central dead zone, the arm moves towards the hand at a fixed slow speed.
- The dead zone uses hysteresis to prevent oscillation near its edge.
- A closed fist, or no hand in view, stops the arm.
- A thumb-index pinch closes the gripper.

A live camera view with the tracking overlay is served on the local network.

## Hardware

- 3D printed arm
- 6 hobby servos on a USB serial controller
- Feetech smart servo for the gripper, with position, temperature and load feedback
- Logitech C920 webcam mounted on the arm
- Server runs on Ubuntu (WSL on Windows)

## Tech stack

Python · Flask · OpenCV · MediaPipe · NumPy · PySerial · JavaScript / HTML Canvas

## Project structure

| File | Purpose |
|---|---|
| `full_servo_hud.py` | Server: IK endpoint, motion control, web panel |
| `robotic_arm_logic.py` | Arm geometry, servo limits, forward and inverse kinematics |
| `servo_controller.py` | Xbox controller client |
| `hand_tracker.py` | Camera capture, hand detection, live preview |
| `hand_control.py` | Hand position → arm movement (tuning settings at the top) |
| `hand_features.py` | Landmark processing and smoothing filters |
| `3d_sim.py` | 3D arm simulator |
| `robot_arm_pick.py` | Colour-based pick and place (work in progress) |

## Getting started

**Server**
```bash
pip install flask pyserial numpy requests
python3 full_servo_hud.py
```
Control panel: `http://localhost:8080`

**Hand tracking** (Python 3.9–3.12)
```bash
pip install mediapipe requests
python hand_tracker.py            # --dry-run to test without moving the arm
```
Camera view: `http://localhost:8090`

**Xbox controller**
```bash
pip install evdev
python3 servo_controller.py
```

Serial ports are configured at the top of `full_servo_hud.py`. Hand-tracking speed, dead zone and axis directions are configured at the top of `hand_control.py`.

## Roadmap

- Correct the shoulder joint range in the IK search, so the arm can reach its full workspace
- Complete colour-based pick and place
- Add distance tracking to hand following
- Add authentication to the web interface

## Authors

<!-- Add your names / GitHub usernames here -->
