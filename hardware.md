# Drivetrain Hardware

This document describes the current [Differential Swerve TeleOp](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DifferentialSwerveTeleOp.java) and [pod encoder diagnostic](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SwervePodEncoderTest.java). See [README.md](README.md) for the operating overview.

## Current Scope

The drive is **robot-centric**, using four motors on the Control Hub and two pod-azimuth encoders on the Expansion Hub. Neither OpMode requires Pinpoint or odometry. The drive has no chassis heading sensor, field reference, or active chassis heading hold.

Earlier Pinpoint mounting and field-centric setup notes describe a superseded configuration and are not prerequisites for these OpModes. SDK samples and SDK support for that hardware are independent of the team's drive implementation.

## Control Hub Motors

| Motor/Encoder Port | Configuration Name | Position |
| --- | --- | --- |
| 0 | `motor0` | Left pod, left motor |
| 1 | `motor1` | Left pod, right motor |
| 2 | `motor2` | Right pod, left motor |
| 3 | `motor3` | Right pod, right motor |

- Four goBILDA 5203-2402-0005 Yellow Jacket motors with 5.2:1 planetary gearboxes.
- Rated no-load output speed: **1150 RPM at 12 V**.
- Built-in encoder resolution: **145.1 ticks per gearbox-output revolution**; 28 at the encoder shaft is not the output-shaft value.
- Each motor encoder connects to the encoder port matching its motor port on the Control Hub.
- All four motors are wired red-to-red/black-to-black and use software direction `FORWARD`.
- Both positive motors propel an aligned pod forward. Left positive/right negative rotates either pod clockwise viewed from above.
- The four built-in encoders provide motor-velocity feedback only. Their positions are not used to estimate pod azimuth, and the OpMode does not reset their counts.

The motors run in `RUN_USING_ENCODER` with velocity PIDF and `BRAKE` zero-power behavior. `setVelocity()` and motor velocity telemetry use ticks/second. The calculated no-load maximum is `1150 * 145.1 / 60 = 2781.08 ticks/s`.

Each motor's velocity contains both wheel-drive and steering components. Equal motor velocities produce wheel drive; their difference produces steering. Motor feedback is therefore not an independent wheel-only sensor.

## Expansion Hub Pod Encoders

| Encoder Port | Configuration Name | Measurement | Clockwise Sign |
| --- | --- | --- | --- |
| 0 | `encoderleft` | Left pod azimuth | Positive |
| 1 | `encoderright` | Right pod azimuth | Positive |

- Model: **REV Through Bore Encoder V1, REV-11-1271**.
- Interface: **quadrature**, not the absolute duty-cycle output.
- Resolution: **2048 cycles/revolution = 8192 decoded counts/revolution**. Do not multiply 8192 by four again.
- Encoder shaft to pod-azimuth gearing: **1:1**.
- Clockwise means looking down from above the robot.
- A 90-degree clockwise rotation should give approximately **+2048 counts**. The measured left-pod result was +2063 counts, equivalent to 90.66 degrees. Both signs were confirmed positive; no resolution adjustment was inferred from that manual test.
- Rolling the drive wheel without rotating the pod should not change these counts.

In Robot Configuration, assign `encoderleft` and `encoderright` to Expansion Hub **motor channels 0 and 1**. The SDK exposes quadrature encoder inputs through those channels even with no motors attached. These channel handles are read-only in both OpModes: no output, mode, direction, or hardware-reset commands are sent to them.

Both OpModes read raw controller counts, bypassing motor-channel direction adjustments. The drive checks that the encoders share a controller on ports 0/1 and are on a different controller from the drive motors. Confirm the actual hub assignment in Robot Configuration; names alone do not establish physical wiring.

### Forward Zero

Quadrature feedback is relative: an encoder does not know that the wheel faces forward. Align both pods forward **before pressing Start**. The drive captures the current counts as the zero baseline and tracks wrapped azimuth using:

```text
delta angle (radians) = delta encoder counts * 2 * PI / 8192
```

Counter differences are computed as integers to handle signed rollover. Pod angular rate is calculated from those differences and elapsed loop time. The motor-encoder gearing calculation from earlier versions is no longer used for steering feedback.

## Mechanical Geometry

| Parameter | Confirmed Value |
| --- | --- |
| Wheel diameter | 60 mm |
| Pod wheel-center spacing | 362.96 mm |
| Pod locations | Directly left/right of robot center |
| Initial bevel pair, each motor | 1:1 |
| Spur stage | 16-tooth driving 54-tooth |
| Wheel bevel stage | 50-tooth driving 19-tooth |
| Wheel-drive ratio | `(16/54) * (50/19)` |

Each motor drives a horizontal-to-vertical 1:1 bevel pair followed by the 16:54 stage. The right motor feeds the upper differential part; the left motor feeds the lower part. The upper/lower 54-tooth gears are rigidly attached to their 50-tooth bevel gears, which engage the 19-tooth wheel gear.

The theoretical no-load wheel speed is approximately **2.817 m/s**. This is used to convert chassis rotation requests into normalized pod speeds; it is not a measured loaded operating limit.

## Controls and Motor Mixing

- Left-stick direction selects robot-relative travel direction; distance from center selects speed after radial deadband rescaling.
- Right-stick X commands rotation about the pod midpoint: right is clockwise, left is counterclockwise. Rotation input is cubed after its deadband.
- Releasing right-stick X commands zero chassis rotation, with no heading hold or return-to-forward behavior.
- Triggers are unused.
- At zero requested pod velocity, its previous azimuth target is retained. Steering can remain active to maintain that pod target.

Per pod, the controller optimizes the requested vector to at most 90 degrees of steering error, reversing wheel direction when appropriate. It applies PD steering with measured-rate damping, a steering-command slew limit, and cosine-squared alignment scaling for wheel drive.

```text
left motor velocity  = wheel-drive component + steering component
right motor velocity = wheel-drive component - steering component
```

Steering has priority within the motor-speed limit; drive uses the remaining headroom. This avoids independently clipping the motor commands and changing the requested steering component. Driving and steering cannot simultaneously demand each motor's full capacity.

## Preliminary Tuning

These are current source defaults, **not validated gains or safe operating limits**:

| Setting | Value | Meaning |
| --- | --- | --- |
| `STEERING_KP` | 2.0 | Normalized steering output per radian of error |
| `STEERING_KD` | 0.003 | Measured-rate damping, normalized output per rad/s |
| `MAX_DRIVE_POWER` | 1.0 | Wheel-drive command scale |
| `MAX_STEER_POWER` | 1.0 | Steering component limit |
| `DRIVE_DEADBAND`, `TURN_DEADBAND` | 0.05 | Stick deadbands |
| `TURN_INPUT_SCALE` | 0.7 | Manual rotation scaling |
| `MAX_TURN_RATE_RADIANS_PER_SECOND` | 3.0 | Rate before manual scaling; full stick requests 2.1 rad/s |
| `STEERING_SLEW_RATE` | 8.0/s | Normalized steering-command change limit; nonpositive disables it |
| `VEL_PID_KP`, `VEL_PID_KI`, `VEL_PID_KD` | 10.0, 0.0, 0.0 | REV motor velocity controller gains |
| `VEL_PID_KF` | `32767 / 2781.0833`, about 11.782 | Preliminary REV velocity feedforward, not volts/RPM |

Public static tuning fields are available in the class, and changed motor PIDF values are reapplied during the telemetry interval. **FTC Dashboard is not installed and the class is not registered with it.** Live Dashboard editing requires a separate dependency/configuration change.

## Commissioning and Faults

1. Securely raise the wheels and reduce drive/steering limits before initial powered testing. Keep clear of moving pods.
2. Run **Swerve Pod Encoder Test**. Tap A with both pods forward, then rotate each pod 90 degrees clockwise and check for approximately +2048 counts. A changes software baselines only; it does not reset hardware encoders. The test works during INIT and after Start and never commands motor outputs.
3. Confirm both wheel-forward directions, clockwise-positive pod feedback, and that wheel rolling alone does not change pod counts.
4. Align both pods forward and start **Differential Swerve TeleOp**. INIT commands zero velocity; it does not disable motors for free manual alignment.
5. Test forward/reverse, strafing, clockwise/counterclockwise turning, combined motion, and turn-stick release at low speed.
6. Test pod-angle wraparound, shortest-path reversals near 90 degrees, and Stop behavior. Tune velocity control, steering PD, slew, and driver feel before increasing limits.

The drive reads one validated bulk snapshot per configured hub per loop and updates telemetry at 10 Hz. A detected invalid snapshot or excessive loop delay stops driving and requires a restart. Shutdown attempts zero velocity on every drive motor, even if another stop command throws, and restores prior hub caching modes. The encoder diagnostic also temporarily uses manual caching and restores prior modes at Stop.

Software checks cannot interrupt a blocked SDK call or guarantee delivery of a failed stop command. A disconnected or frozen quadrature encoder can still return valid hub data and is not reliably detected. Because every configured hub is checked, an unrelated hub fault can also stop the drive or suppress a diagnostic reading.

APK builds and simulated regression tests have passed. Physical PIDF/PD tuning, loaded speed, stopping behavior, and actual Control Hub loop timing remain to be verified on the robot. No Pinpoint commissioning is needed for this setup.
