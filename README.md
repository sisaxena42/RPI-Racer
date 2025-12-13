# Fish Car - Fish Operated Vehicle (FOV)

A robotic RC car controlled by a live fish swimming in a tank. The fish's position is tracked via computer vision, translated into movement commands by a Raspberry Pi, and executed by an Arduino controlling the car's steering and motor.

---

## 🎯 Project Overview

The Fish Car uses real-time computer vision to track a fish's position in a tank and translates that position into vehicle control commands. The system features:

- **Computer Vision Tracking**: HSV color-based fish detection with smoothing
- **Safety Systems**: LIDAR obstacle detection and virtual boundary enforcement
- **Smooth Control**: Gradual acceleration/deceleration and steering transitions
- **Dead Reckoning**: Position estimation for boundary management
- **Real-time Processing**: 50Hz control loop with minimal latency

---

## 🏗️ System Architecture

```
┌─────────────┐
│   Camera    │ (Above tank, pointing down)
└──────┬──────┘
       │ USB
       ▼
┌─────────────────────────────────────┐
│      Raspberry Pi 4/5               │
│  • Computer Vision (OpenCV)         │
│  • Fish Detection & Tracking        │
│  • LIDAR Processing                 │
│  • Safety Logic                     │
│  • Command Generation               │
└──────┬──────────────────────────────┘
       │ USB Serial (19200 baud)
       ▼
┌─────────────────────────────────────┐
│         Arduino Uno                 │
│  • Command Parsing                  │
│  • Smooth Transitions               │
│  • PWM Signal Generation            │
└──────┬──────────────────────────────┘
       │
       ├─────► Servo (Pin 14) ────► Front Wheels
       │
       └─────► ESC (Pin 9) ──────► DC Motor
```

### Hardware Components

- **Raspberry Pi 4/5**: Runs Python control software with OpenCV
- **Arduino Uno**: Generates precise PWM signals for servo and ESC
- **USB Camera**: Mounted above tank for fish tracking (640×480 @ 30fps)
- **Hokuyo UST-10LX LIDAR**: Front-facing obstacle detection (270° FOV)
- **Steering Servo**: Controls front wheel angle (60°-120° range)
- **Brushless DC Motor + ESC**: Drives rear wheels (forward/reverse)
- **RC Receiver**: Power distribution and signal reference
- **LiPo Battery**: Powers entire system via ESC/BEC

---

## 📁 Repository Structure

```
fishycode/
├── fish_car_controller.py          # Main Pi control script (CURRENT)
├── arduino/
│   ├── fish_car_smooth_control.ino # Main Arduino firmware (CURRENT)
│   └── motion_control.ino          # Legacy Arduino firmware
├── motion_tracking/
│   ├── fish_tracker.py             # Simplified tracking script
│   ├── tracking_fish.py            # Alternative tracker (9600 baud)
│   └── color_selection.py          # HSV calibration tool
├── tests/
│   ├── lidar_test.py               # LIDAR connection test
│   ├── lidar_stop_car.py           # LIDAR safety test
│   ├── motor_test.py               # ESC calibration test
│   ├── servo_test.py               # Servo range test
│   └── serial_*.py                 # Serial communication tests
└── misc/
    └── *.ino                       # Hardware bring-up sketches
```

---

## 🔄 Control Flow

### 1. Fish Detection (Raspberry Pi)
```python
Camera → HSV Conversion → Color Thresholding → Contour Detection → Centroid Calculation
```

- Captures 640×480 frames at ~30 FPS
- Converts to HSV color space for robust color detection
- Applies calibrated HSV bounds to isolate fish
- Finds largest contour and calculates centroid
- Applies exponential smoothing to reduce jitter

### 2. Safety Checks (Raspberry Pi)
```python
LIDAR Scan → Front Obstacle Detection → Virtual Boundary Check → Speed Adjustment
```

- **LIDAR**: Scans ±30° in front, stops if obstacle < 0.7m
- **Virtual Boundary**: Tracks position via dead reckoning, enforces 3m radius
- **Speed Reduction**: Gradually reduces speed when approaching boundary (2.5m-3.0m)

### 3. Command Generation (Raspberry Pi)
```python
Fish Position → Grid Mapping (3×3) → Speed/Steering Calculation → Serial Transmission
```

Command format: `x,y,speed_factor,boundary_factor,obstacle\n`
- `x,y`: Fish position in pixels (0-640, 0-480)
- `speed_factor`: 0.0-1.0 (safety-adjusted speed multiplier)
- `boundary_factor`: 0.0-1.0 (proximity to boundary)
- `obstacle`: 0=clear, 1=obstacle detected

### 4. Motion Control (Arduino)
```arduino
Serial Parse → Grid Sector → Target Calculation → Smooth Transition → PWM Output
```

**Grid Mapping (3×3):**
```
┌─────────┬─────────┬─────────┐
│ FWD+L   │ FORWARD │ FWD+R   │  Top: Fish far → Move forward
├─────────┼─────────┼─────────┤
│ SLOW+L  │  IDLE   │ SLOW+R  │  Mid: Fish medium → Slow forward
├─────────┼─────────┼─────────┤
│ REV+L   │ REVERSE │ REV+R   │  Bot: Fish close → Move backward
└─────────┴─────────┴─────────┘
  Left      Center     Right
```

**Smoothing:**
- Speed: 15% change per iteration (prevents jerky acceleration)
- Steering: 20% change per iteration (smooth turns)
- Max changes: ±5% speed, ±3° steering per 20ms loop

**ESC Calibration:**
- Neutral: 1388 µs (stopped)
- Forward: 1388-1530 µs (max 7% power for safety)
- Reverse: 1064-1388 µs (max 20% power)

---

## 🚀 Getting Started

### Prerequisites

**Hardware:**
- Raspberry Pi 4/5 with Raspbian/Ubuntu
- Arduino Uno with USB cable
- USB webcam
- Hokuyo UST-10LX LIDAR (Ethernet connection)
- RC car chassis with servo and ESC
- Fish tank with clear water and good lighting

**Software:**
```bash
# Raspberry Pi
sudo apt update
sudo apt install python3-opencv python3-serial python3-numpy

# Install LIDAR library
pip3 install hokuyolx

# Arduino IDE
# Install from https://www.arduino.cc/en/software
```

### Installation

1. **Clone Repository:**
```bash
git clone <repository-url>
cd fish_car
```

2. **Configure LIDAR Network:**
```bash
# Set static IP for LIDAR connection
# Edit /etc/network/interfaces or use NetworkManager
# LIDAR default IP: 192.168.0.10
```

3. **Calibrate Fish Color:**
```bash
python3 fishycode/motion_tracking/color_selection.py
```
- Adjust HSV trackbars until only fish is visible in mask
- Note the HSV values
- Update `LOWER_HSV` and `UPPER_HSV` in [`fish_car_controller.py`](fishycode/fish_car_controller.py:30-32)

4. **Flash Arduino:**
- Open [`fish_car_smooth_control.ino`](fishycode/arduino/fish_car_smooth_control.ino) in Arduino IDE
- Verify pin assignments match your hardware
- Upload to Arduino Uno

5. **Test Components:**
```bash
# Test LIDAR
python3 fishycode/tests/lidar_test.py

# Test servo (wheels off ground!)
python3 fishycode/tests/servo_test.py

# Test motor (wheels off ground!)
python3 fishycode/tests/motor_test.py
```

### Running the System

1. **Setup:**
   - Mount camera directly above tank pointing down
   - Ensure stable lighting (reduces HSV jitter)
   - Place LIDAR facing forward on car
   - Connect all hardware and power on

2. **Start Control System:**
```bash
python3 fishycode/fish_car_controller.py
```

3. **Operation:**
   - System initializes (camera, serial, LIDAR)
   - Place fish in tank
   - Car will follow fish movements
   - Press 'q' to quit safely

---

## ⚙️ Configuration

### Camera Alignment ([`fish_car_controller.py`](fishycode/fish_car_controller.py:21-27))
```python
FLIP_X = False      # Mirror left/right if camera is reversed
FLIP_Y = False      # Mirror up/down if camera is inverted
ROTATE = 0          # Rotate 0/90/180/270 degrees
X_OFFSET = 0        # Shift steering center (pixels)
```

### HSV Color Bounds ([`fish_car_controller.py`](fishycode/fish_car_controller.py:30-32))
```python
LOWER_HSV = np.array([22, 95, 169])   # Adjust for your fish
UPPER_HSV = np.array([179, 255, 255])
```

### Safety Parameters ([`fish_car_controller.py`](fishycode/fish_car_controller.py:34-43))
```python
LIDAR_STOP_DISTANCE = 0.7        # Stop if obstacle within 0.7m
LIDAR_FRONT_ANGLE_DEG = 30       # Check ±30° in front
BOUNDING_BOX_RADIUS = 3.0        # Max 3m from start position
BOUNDING_BOX_WARNING_RADIUS = 2.5 # Start slowing at 2.5m
ENABLE_BOUNDING_BOX = True       # Enable/disable boundary
```

### Motion Parameters ([`fish_car_controller.py`](fishycode/fish_car_controller.py:45-50))
```python
SMOOTHING_FACTOR = 0.3           # Fish position smoothing (0-1)
MIN_FISH_DETECTION_AREA = 100    # Min contour area (pixels²)
LOOP_DELAY = 0.02                # 50Hz control loop
```

### Arduino Tuning ([`fish_car_smooth_control.ino`](fishycode/arduino/fish_car_smooth_control.ino:24-46))
```cpp
// Speed limits (percentage)
MAX_MOTOR_SPEED = 7;        // Forward speed (safety limited)
MAX_REV_SPEED = 20;         // Reverse speed
MIN_MOTOR_SPEED = 6;        // Minimum to overcome friction

// ESC calibration (measure with receiver)
ESC_MIN  = 1064;  // Full reverse
ESC_NEUT = 1388;  // Neutral/stopped
ESC_MAX  = 1530;  // Full forward

// Steering angles
STEERING_CENTER = 90;
STEERING_LEFT = 60;
STEERING_RIGHT = 120;

// Smoothing
SPEED_SMOOTHING = 0.15;     // Lower = smoother
STEERING_SMOOTHING = 0.20;
```

---

## 🔧 Troubleshooting

### Fish Not Detected
- Run [`color_selection.py`](fishycode/motion_tracking/color_selection.py) to recalibrate HSV bounds
- Check lighting conditions (avoid shadows, reflections)
- Ensure camera is focused and clean
- Verify `MIN_FISH_DETECTION_AREA` isn't too high

### Car Not Moving
- Check serial connection: `ls /dev/ttyACM*`
- Verify ESC is armed (3-second delay in Arduino setup)
- Test motor separately with [`motor_test.py`](fishycode/tests/motor_test.py)
- Check battery voltage (LiPo should be >11V)
- Verify ESC calibration values match your hardware

### Jerky Movement
- Increase `SMOOTHING_FACTOR` in Python (0.3 → 0.5)
- Increase `SPEED_SMOOTHING` in Arduino (0.15 → 0.25)
- Reduce `MAX_MOTOR_SPEED` for gentler acceleration
- Check for camera frame drops (should be ~30 FPS)

### LIDAR Not Working
- Verify network connection: `ping 192.168.0.10`
- Check LIDAR power and Ethernet cable
- Run [`lidar_test.py`](fishycode/tests/lidar_test.py) for diagnostics
- Ensure LIDAR IP matches `LIDAR_IP` in config

### Boundary Not Working
- Dead reckoning drifts over time (expected)
- Reset by restarting the program
- Adjust `BOUNDING_BOX_RADIUS` for your space
- Disable if not needed: `ENABLE_BOUNDING_BOX = False`

---

## 📊 Performance Metrics

- **Control Loop**: 50 Hz (20ms per iteration)
- **Camera**: 30 FPS @ 640×480
- **LIDAR**: ~40 Hz scan rate
- **Serial**: 19200 baud (sufficient for ~960 commands/sec)
- **Latency**: ~50-100ms end-to-end (camera → Arduino → motion)

---

## 🛠️ Development Tools

### Test Scripts
- [`lidar_test.py`](fishycode/tests/lidar_test.py) - Verify LIDAR connection and scan data
- [`motor_test.py`](fishycode/tests/motor_test.py) - Test ESC response and calibration
- [`servo_test.py`](fishycode/tests/servo_test.py) - Verify steering range
- [`serial_com_from_pi.py`](fishycode/tests/serial_com_from_pi.py) - Test Pi → Arduino communication
- [`serial_com_from_arduino.py`](fishycode/tests/serial_com_from_arduino.py) - Test Arduino → Pi communication

### Calibration Tools
- [`color_selection.py`](fishycode/motion_tracking/color_selection.py) - Interactive HSV calibration
- [`misc/calibration.ino`](fishycode/misc/calibration.ino) - ESC calibration routine
- [`misc/servo_test.ino`](fishycode/misc/servo_test.ino) - Servo range finder

### Debug Sketches
- [`misc/motor_test.ino`](fishycode/misc/motor_test.ino) - Basic motor control
- [`misc/backwards.ino`](fishycode/misc/backwards.ino) - Reverse motion test
- [`misc/drive_circle.ino`](fishycode/misc/drive_circle.ino) - Circular motion test

---

## 🔐 Safety Features

1. **Command Timeout**: Arduino stops if no command received for 1 second
2. **LIDAR Obstacle Detection**: Stops if obstacle within 0.7m
3. **Virtual Boundary**: Prevents car from traveling >3m from start
4. **Speed Limiting**: Maximum 7% forward power for safety
5. **Smooth Transitions**: Prevents sudden jerks that could damage hardware
6. **Emergency Stop**: Press 'q' to safely shut down system

---

## 📝 Technical Notes

### Why Arduino + Raspberry Pi?
- **Pi**: High-level processing (OpenCV, LIDAR, safety logic)
- **Arduino**: Deterministic PWM timing (no Linux jitter)
- **Result**: Smooth servo/ESC control with intelligent decision-making

### Dead Reckoning Limitations
- Position estimate drifts over time (wheel slip, turning radius approximation)
- Boundary enforcement is approximate, not GPS-accurate
- Suitable for short-duration runs (<5 minutes)
- Consider adding wheel encoders or IMU for better accuracy

### ESC Behavior
- Requires neutral pulse (1388 µs) for 3 seconds to arm
- Forward/reverse transition requires brief neutral period
- Different ESCs may have different calibration values
- Measure your ESC's actual values with receiver before use

---

## 🚧 Future Enhancements

- [ ] Add wheel encoders for accurate odometry
- [ ] Implement IMU for heading correction
- [ ] Add GPS for absolute positioning
- [ ] Create web interface for remote monitoring
- [ ] Add data logging for analysis
- [ ] Support multiple fish (swarm behavior)
- [ ] Add obstacle avoidance path planning
- [ ] Implement PID control for smoother motion

---

## 📄 License

This project is open source. Feel free to modify and adapt for your own fish-controlled vehicles!

---

## 🙏 Acknowledgments

- OpenCV community for computer vision tools
- Hokuyo for LIDAR hardware and Python library
- Arduino community for servo/ESC control examples

---

## 📧 Support

For issues, questions, or contributions, please open an issue in the repository.

**Happy Fish Driving! 🐟🚗**
