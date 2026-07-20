# AI-Powered Embedded Tracking Turret

A 2-DOF pan/tilt turret that detects a person with an on-board AI model and physically keeps a laser on them as they move. Runs on the Arduino UNO Q. A Kalman filter smooths the noisy detections and predicts where the target is going, and a PD controller drives the servos to get there without overshooting.

<!-- Drop your demo GIF or video here. This is the most important thing on the page. -->
![Demo](docs/demo.gif)

---

## What it does

The camera watches the room. When a person walks in, the AI model draws a box around them. The system takes the center of that box, filters it, figures out how fast the person is moving, and moves two servos so the laser lands on them. As they walk, it follows.

The interesting part isn't the detection. It's everything after it. The AI only reports a new position about 10 to 15 times a second, and those positions jitter by a few pixels even when nobody's moving. Feeding that straight to the servos makes them twitch and stutter. So the real work was the estimation and control that sits between the camera and the servos.

---

## Hardware

| Part | What it does |
|---|---|
| Arduino UNO Q (4 GB) | One board, two processors. Linux side runs the vision and control in Python, MCU side generates the servo pulses. |
| Logitech C920s | USB camera, 640 x 480 detection frame |
| Yahboom YB-P25M | 2-DOF metal pan/tilt gimbal |
| KY-008 laser module | Mounted with the camera, this is what points at the target |

**Pins:** bottom servo on 10, top servo on 9, laser on 7.

The camera and laser are both mounted on the tilt bar so they move together. That keeps the aim aligned with what the camera sees no matter where the rig sits. The tradeoff is that moving the servo also moves the camera, which caused a real problem I'll get to below.

---

## How it works

```
camera frame
    -> AI detection (bounding box)
    -> centroid (one point to aim at)
    -> Kalman filter (smooth + predict)
    -> error vs laser center
    -> PD controller
    -> servo positions over the Bridge
    -> MCU generates pulses
```

Python does the thinking on the Linux side. The MCU does the muscle. They talk over the Arduino RouterBridge.

### Detection to a target point

The AI gives back a box as four numbers. To get one point to aim at, average the opposing edges:

```
cx = (x_min + x_max) / 2
cy = (y_min + y_max) / 2
```

Sometimes more than one box shows up and some of them are wrong. Furniture would occasionally read as a person. So the system only tracks the largest box above a minimum area, which throws out the small false ones.

### The laser isn't at the center of the image

This one caught me off guard. The laser is mounted above the camera, not on the lens, so it doesn't point at the middle of the frame. That's parallax. If you aim at the geometric center, which is (320, 240) on a 640 x 480 frame, the dot always lands off the person.

So I calibrated it instead of assuming. Stood so the laser dot landed on the middle of my body, then read what centroid the system reported at that moment. That pixel became the laser center. It came out to (300, 240), about 20 pixels off.

```
error_x = centroid_x - LaserCenterX
error_y = centroid_y - LaserCenterY
```

### Servo calibration

The servos are driven by microsecond pulse widths. The width sets the position. It maps to an angle but it's not a clean microseconds-equals-degrees conversion, and every servo's range is a little different, so I couldn't just use the standard values.

Normally you'd use the Arduino Servo library, but it doesn't compile for this board's STM32 target. So I generate the pulses manually: set the pin HIGH, wait N microseconds, set it LOW. That meant finding the limits myself by sweeping the pulse width and watching where each servo physically stopped.

| Servo | Min | Center | Max |
|---|---|---|---|
| Bottom (pan) | 375 | 1275 | 2175 |
| Top (tilt) | 500 | 1275 | 2050 |

---

## Kalman filter

The measurement isn't the truth. The box jitters, it only arrives 10 to 15 times a second, and once in a while the AI drops the person for a frame. The filter keeps its own estimate of position and velocity and blends a prediction with each new measurement.

Four steps, every frame:

```
Predict:   PredictedPosition = position + velocity
Residual:  Residual = new_measurement - PredictedPosition
Correct:   NewPosition = PredictedPosition + camTrust * Residual
Learn:     NewVelocity = velocity + velTrust * Residual
```

The part I like: velocity is never measured. It gets learned from the residuals. If the target keeps landing ahead of where the filter predicted, the filter figures out it's moving and builds up a velocity. That velocity feeds the predict step, so the estimate leads the motion instead of trailing it.

### Worked example

Target moving right at 20 pixels per frame, with camTrust = 0.8 and velTrust = 0.2:

| Frame | Measured | Predicted | Estimate | Velocity |
|---|---|---|---|---|
| 1 | 350 | — | 350.0 | 0.0 |
| 2 | 370 | 350.0 | 354.0 | 8.0 |
| 3 | 390 | 362.0 | 367.6 | 19.2 |
| 4 | 410 | 386.8 | 391.4 | 28.5 |

The measurement jumps in hard 20-pixel steps. The estimate ramps up smoothly instead. Velocity starts at zero and converges toward the true 20.

### The two dials

- **camTrust** — how much to trust the camera against the filter's own prediction. High is responsive but jittery, low is smooth but slow to react.
- **velTrust** — how fast the velocity estimate adapts. High reacts to speed changes quickly but overshoots, low is stable but slower to lead.

I landed on 0.8 and 0.2. I tried the opposite and the position lagged behind while the velocity overshot and oscillated. Trusting the measurement more and letting velocity build slowly gave a responsive position with a stable velocity.

---

## PD control

Once there's a clean estimate, the PD controller turns the error into servo movement:

```
movement = PUSH_STRENGTH * error - BRAKE_STRENGTH * (change in error)
```

**P is the push.** It's proportional, so it turns pixels of error into microseconds of servo travel. Far off target means a big move, close means a small one. On its own it overshoots and oscillates every time.

**D is the brake.** It watches how fast the error is changing. When the laser is closing in fast or starting to overshoot, that rate is large, so the brake pushes back and damps the motion. As things settle and the error barely changes, the brake fades to almost nothing. It's not about how big the error is, it's about how fast it's changing. That's what kills the overshoot.

There's also a dead zone. If the error is small enough, it gets zeroed so the servo doesn't twitch chasing a couple pixels of noise.

One thing worth knowing: the code sends absolute positions, not directions. The servo always drives toward whatever position it's given. If the new value is lower than where it currently is, it turns the other way on its own. You never command a direction, just where to be.

---

## Challenges

**Runaway feedback loop.** The camera is mounted on the gimbal, so moving the servo moves the camera too. With the feedback sign backwards, every correction made the error grow instead of shrink and the gimbal drove itself into its end stop. I found it by logging the error every frame and watching it swing plus or minus 350 pixels while I stood still, which is the signature of positive feedback. The fix was working out the sign convention: the direction of the error subtraction and the servo sign constant together set the loop polarity, and you have to flip exactly one of them.

**Detection rate is a hardware ceiling.** The AI only runs 10 to 15 times a second. That's not a software problem. The UNO Q's processor is a small low-power embedded chip, roughly Raspberry Pi 3 class, with no dedicated AI accelerator, and running a full object-detection network takes 66 to 100 ms per frame. Meanwhile the MCU can update the servos at 66 to 200 Hz. Since new targets only arrived 10 to 15 times a second, the servo would jump to each one and pause, which showed up as visible stutter.

**Two smoothing layers fighting each other.** Early on both the MCU and the Python side were smoothing. Stacked together they made the response sluggish and then it would overshoot anyway. Fixed by keeping all the control logic in Python and letting the MCU just execute.

**Stutter fix.** The MCU doesn't snap to each new target. It glides toward it a fraction each fast loop, filling the gaps between detections so the slow updates blend into continuous motion. The MCU stays fast, it just interpolates between the slow targets. That plus the Kalman filter predicting through the gaps is what made the tracking smooth despite the slow detection rate.

**False detections.** Furniture would sometimes get classified as a person and the laser would jump to it. Raised the confidence threshold and only track the largest box above a minimum area.

---

## Improvements

- **Lighter model.** A person-only model would run faster than the general object detector and raise the frame rate, which is the main limit on smoothness.
- **Offload the detection.** The real bottleneck is the detection rate, not the MCU. Running the AI on a processor with actual AI acceleration would raise the detection rate itself.
- **Velocity lead.** The filter already estimates velocity, so it could aim where the target is going instead of where it was, which cancels some of the latency.
- **Distance-based gains.** The pixel-to-angle relationship changes with range, so scheduling the gain against estimated distance would keep the response consistent as the target moves closer or further.
- **Servos with encoders.** These servos give no position feedback, so the loop closes on the commanded angle, not the actual one.

---

## Running it

The Python side lives in `python/main.py` and runs on the Linux processor. The MCU sketch lives in `sketch/sketch.ino`. The app needs the Video Object Detection brick and the Web UI brick.

Tuning values that worked for me:

```
PUSH_STRENGTH  = 0.1
BRAKE_STRENGTH = 0.1
DEAD_ZONE      = 40
MIN_BOX_AREA   = 10000
camTrust       = 0.8
velTrust       = 0.2
SMOOTH (MCU)   = 0.15
```

Your laser center and servo limits will be different. Calibrate those first or nothing will land where you expect.

---

## Why I built it

This started as a course project for ECE 5520 Automotive Mechatronics, but the reason it's interesting is that it's the same problem real tracking systems solve. A Kalman filter estimating target position and velocity from noisy sensor data is what ADAS uses for pedestrian tracking and what radar systems use for target tracking. PD control driving actuators to hold a target is turret and gimbal stabilization. Perception, estimation, control, actuation. This is a small version of that pipeline where I had to build the estimation and control parts myself.
