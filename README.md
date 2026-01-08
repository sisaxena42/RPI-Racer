# RPI-Racer (Model 1)

Model 1 of the RPI-Racer project is a foundational prototype used to validate Arduino-based control of an RC chassis using commands issued from a Raspberry Pi. This version focuses on **servo steering**, **motor + ESC throttle control**, and **stable serial communication** between the Pi and Arduino. Unlike Model 2, Model 1 does **not** include computer vision or fish tracking; all motion is triggered manually through Python scripts.

Model 1 served as the hardware bring-up platform before integrating real-time vision and behavioral logic in later versions.

---

## System overview

- **Arduino:** Receives `"RUN"` triggers from the Pi over USB serial. Runs simple motion sequences defined in test sketches (`servo_test.ino`, `motor_test.ino`).
- **Servo:** Controls the front wheel steering. Connected to Arduino pin 14.
- **Motor + ESC:** Controls vehicle drive. ESC signal line connected to Arduino pin 9.
- **Receiver + Power:** Standard RC receiver distributes power to both the servo and ESC. The Arduino shares the 5V/GND rail to ensure common reference.
- **Raspberry Pi:** Runs Python scripts in `fishycode/tests/` to send serial commands and validate behavior.

All motion logic in Model 1 is hardcoded on the Arduino and triggered by the Pi.

---

## Repository layout
```
fishycode/
├─ arduino/ → Core Arduino sketches (servo_test.ino, motor_test.ino)
├─ tests/ → Python scripts to trigger Arduino movement via serial
├─ motion_tracking/ → Placeholder; unused in Model 1
└─ misc/ → Early sketches and bring-up utilities
```

---

## Control loop (Model 1)

1. **Flash Arduino with test sketch**  
   Upload either `servo_test.ino` or `motor_test.ino` using the Arduino IDE.

2. **Run Python trigger script**  
   Execute `servo_test.py` or `motor_test.py` on the Raspberry Pi. This opens a serial connection and sends `"RUN\n"`.

3. **Arduino executes motion sequence**  
   The sketch checks for `"RUN"` and runs the corresponding servo sweep or throttle sequence once.

4. **Car moves and stops**  
   After completing the motion, the Arduino prints `"Done"` and halts further movement.

---

## Software modules

### Arduino sketches (`fishycode/arduino/`)

#### `servo_test.ino`
- Sweeps the steering servo through a fixed pattern (e.g. left → center → right → center).
- Uses:
  - `Servo` library
  - `Serial.readStringUntil('\n')`
  - `hasRun` flag to prevent repeat execution

#### `motor_test.ino`
- Sends incremental throttle commands to the ESC.
- Used for:
  - ESC arming verification
  - Battery validation
  - Motor direction testing

Both sketches operate at **19200 baud**.

---

### Python test scripts (`fishycode/tests/`)

#### `servo_test.py`
- Opens serial connection to `/dev/ttyACM0`
- Sends `"RUN\n"`
- Prints Arduino responses (e.g. `"Done"`)
- Used to validate steering control from the Pi

#### `motor_test.py`
- Same structure as `servo_test.py`
- Focused on throttle control and ESC behavior

All scripts include:
- `time.sleep(2)` to allow Arduino reset
- Safe `try/finally` cleanup for serial port handling

---

## Hardware integration notes

### Servo (front steering)
- Mounted on front knuckles with standard horn
- Must be mechanically centered at 90°
- Typical operating range: ~60° (full left) to ~120° (full right)

### Motor + ESC
- ESC signal connected to Arduino pin 9
- PWM range:
  - ~1050 µs → full reverse
  - ~1500 µs → neutral
  - ~1950 µs → full forward
- ESC must be armed at neutral before motion

### Receiver + power
- Receiver powers both servo and ESC (via BEC)
- Arduino shares the 5V/GND rail for consistent reference
- Raspberry Pi does **not** power motors or servos

### Raspberry Pi serial
- Typical port: `/dev/ttyACM0`
- May switch to `/dev/ttyACM1` after upload due to USB re-enumeration
- Use `/dev/serial/by-id/` for stable device naming if needed

---

## Known limitations

- No real-time decision making
- All motion is discrete and scripted
- USB port may reset after sketch upload
- Requires external power for servo/motor (Pi cannot supply enough current)

---

## Purpose of Model 1

Model 1 exists to:

- Validate wiring and power distribution
- Confirm ESC calibration and throttle response
- Confirm servo range and steering geometry
- Debug Pi ↔ Arduino serial reliability
- Provide a safe environment for hardware bring-up

All higher-level intelligence is intentionally excluded.

---

## Transition to Model 2

Model 2 builds directly on Model 1 by adding:

- Camera + overhead mounting
- HSV color tracking of fish
- Real-time coordinate streaming
- Region-based behavioral logic
- Smooth steering and throttle transitions

Model 1 remains useful for:
- Regression testing
- Hardware swaps
- Demonstrations without live fish
- ESC and servo calibration

---

## Recreating Model 1

1. Mount servo, ESC, motor, receiver, and Arduino on chassis
2. Connect servo to pin 14, ESC signal to pin 9
3. Upload `servo_test.ino` or `motor_test.ino`
4. Power receiver with RC battery
5. Run `python fishycode/tests/servo_test.py` on the Pi
6. Observe movement and adjust hardware alignment as needed

Once all systems behave correctly, proceed to Model 2 for full fish-operated control.


# RPI-Racer (Model 2)

The Fish Operated Vehicle (FOV) is an RC chassis that lets a live fish “drive” the car. A camera mounted above the tank tracks the fish, a Raspberry Pi translates the motion into steering/throttle commands, and an Arduino pushes those commands to the servo and motor/ESC combo. Model 2 (this repo) is the current working build; Model 1 will be documented when that variant is ready.

---

## System overview

- **Camera:** Mounted above the tank, streams live video to the Raspberry Pi so we always know where the fish is.
- **Raspberry Pi:** Runs the computer-vision stack under `fishycode/motion_tracking`. It isolates the fish by color (HSV filtering), finds the centroid, and streams the `(x,y)` position over serial.
- **Arduino:** Runs `fishycode/arduino/motion_control.ino`. It receives the Pi coordinates, figures out which region of the frame the fish lives in, and converts that to servo angle plus motor power.
- **Servo:** Tied to the front knuckles; `Servo.write()` angles between roughly 60° (full left) and 120° (full right).
- **Motor + Electronic Speed Controller:** ESC interprets the Arduino PWM, spins the motor forward/backward, and therefore moves the car.
- **Receiver:** Powers both the servo and ESC/BEC and keeps the signaling tidy; it is the shared reference for all PWM signals even though the Pi/Arduino now dictate the values.
- **Lidar:** _Placeholder — document lidar wiring/usage here once the integration details are finalized._

All control intelligence is on the Pi, while the Arduino provides deterministic, low-jitter PWM for the servo and ESC.

---

## Repository layout

```
fishycode/
├─ arduino/            → Motion control firmware that drives the servo and ESC
├─ motion_tracking/    → Pi-side computer vision and serial output
├─ tests/              → Small host utilities for talking to the Arduino/ESC
└─ misc/               → One-off Arduino sketches used during bring-up
```

---

## Control loop at a glance

1. **Camera capture:** `fish_tracker.py` grabs frames from `/dev/video0` (or your configured device) at 640×480.
2. **Fish detection:** The frame is converted to HSV, thresholded according to your calibrated color bounds (`color_selection.py` helps dial these in), and the largest contour is assumed to be the fish.
3. **Coordinate broadcast:** The contour’s centroid becomes `center_x,center_y`, which the Pi streams to the Arduino over USB serial at 19.2 kbps.
4. **Region mapping:** The Arduino divides the 1280×1080 virtual frame into a 3×3 grid. Depending on the sector the fish occupies, it chooses one of nine behaviors (idle, turn left/right while moving forward/back, etc.).
5. **Actuation:** `steering_servo.write()` angles the wheels, while `drive_motor.writeMicroseconds()` sends ESC pulses mapped from -100–100% power to 1050–1950 µs.
6. **Vehicle motion:** The ESC powers the DC motor, the receiver distributes power/signals to both the servo and ESC, and the car mirrors the fish’s heading.

When the fish remains in the central sector, power drops to zero and the servo straightens—effectively parking the car until the fish swims elsewhere.

---

## Software modules

### Motion tracking (`fishycode/motion_tracking/`)
- `color_selection.py`: Opens OpenCV trackbars so you can interactively find HSV ranges that isolate your specific fish/lighting conditions. Use this before each install or whenever lighting changes drastically.
- `fish_tracker.py`: The primary Pi script. Handles camera setup, HSV masking, contour selection, debug overlays, and serial writes to the Arduino (default `/dev/ttyACM0`, 19200 baud).
- `tracking_fish.py`: Earlier prototype that streams coordinates as `X=..,Y=..`. Useful reference if you need a simpler output format or a different baud rate (9600).

### Arduino motion control (`fishycode/arduino/motion_control.ino`)
- Uses the standard Servo library to attach the steering servo on pin 14 and the ESC signal on pin 9.
- Subscribes to serial messages formatted as `x,y`. The sketch trims input, validates the comma, then hands the parsed integers to `handleMovement`.
- `handleMovement` determines the grid sector, translates that to forward/reverse throttle (`setDrivePower`) plus a discrete steering angle (`steerByColumn`). Movement logic currently favors constant-speed movements (`MOTOR_SPEED` = 40) with distinct rules for top/middle/bottom thirds of the frame.
- `setDrivePower` clamps ±100% and linearly maps to ESC microseconds (1050–1950). This keeps output compatible with hobby ESCs expecting 1–2 ms pulses.

### Hardware tests (`fishycode/tests/`)
- `servo_test.py` / `servo_test_updated.py`: Sends a `RUN` trigger, opens serial, and prints the Arduino responses so you can verify that the steering and ESC arming routine behave on the bench.
- `motor_test.py`: Same idea but focused on throttle sweep sequences (neutral → forward power). Handy when validating ESC calibration or battery health.
- `serial_*` helpers: Quick sanity checks to confirm the Pi can open the correct serial device in both directions.

### Prototype Arduino sketches (`fishycode/misc/`)
These sketches exist as historical bring-up utilities—sweeping servos, looping the motor in circles, or moving the chassis back and forth. They are a good starting point when swapping hardware (e.g., a different ESC) before flashing the full `motion_control.ino`.

---

## Hardware integration notes

- **Servo (front wheels):** Responsible for lateral movement. Because steering is discrete in the firmware (left/center/right), align the horn mechanically so 90° equals straight wheels.
- **Motor + ESC:** ESC interprets the PWM pulses from pin 9. Ensure the ESC is armed at neutral (≈1500 µs) before issuing throttle, as shown in `motor_test.ino` and the Python test clients.
- **Receiver:** Acts as the power distribution board. It powers both the servo and the ESC/BEC, and the Arduino piggybacks on the receiver’s 5 V/GND rails so every signal shares a reference.
- **Camera:** Place directly above the tank pointing down. Stable lighting reduces HSV jitter, which in turn keeps the car from twitching.
- **Arduino vs. Pi:** The Pi does the high-level “where is the fish?” computation; the Arduino guarantees precise PWM without Linux jitter affecting the servo or ESC timing.

---

## Lidar integration (to be documented)

_TODO: Add wiring diagram, control strategy, and any supporting code for the lidar once Model 2 testing validates it._

---

## Roadmap and Model 1

This document covers Model 2, which is the current fish-controlled prototype. Model 1 (an earlier hardware stack) will be documented in this README once that version is ready to release so we can compare the two approaches.

---

## Recreating the project

1. **Assemble hardware:** Mount camera, Pi, receiver, Arduino, ESC, servo, and DC motor onto the chassis. Verify the receiver distributes power (BEC) to both servo and ESC.
2. **Calibrate the fish color:** Run `python fishycode/motion_tracking/color_selection.py` on the Pi, note the HSV values that best isolate the fish, and plug them into `fish_tracker.py`.
3. **Flash the Arduino:** Upload `fishycode/arduino/motion_control.ino`. Confirm steering direction and ESC arming while the wheels are off the ground.
4. **Test links:** Use the scripts in `fishycode/tests/` to make sure the Pi ↔ Arduino serial connection is stable and that servo/motor respond before introducing the fish.
5. **Run the loop:** Start `python fishycode/motion_tracking/fish_tracker.py`, then place the fish in the tank. The Pi will stream positions, the Arduino will steer/throttle, and the car will mimic the fish’s heading in real time.

With these pieces in place, the fish effectively “drives” the vehicle—the FOV responds to whichever direction the fish swims toward inside the tank.
