# Drivetrain Hardware

This document describes the current [Differential Swerve TeleOp](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DifferentialSwerveTeleOp.java) and [pod encoder diagnostic](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SwervePodEncoderTest.java). See [README.md](README.md) for the operating overview.

## Current Scope

The drive is **robot-centric**, using four motors on the Control Hub and two Melonbotics pod-azimuth encoders with analog outputs on the Control Hub and quadrature outputs on the Expansion Hub. Neither OpMode requires Pinpoint or odometry. The drive has no chassis heading sensor, field reference, or active chassis heading hold.

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
- The controller assumes both positive motors propel an aligned pod in its directed robot-forward travel direction, and left positive/right negative rotates either pod clockwise viewed from above. Physically validate both assumptions on each module; forward encoder references do not validate motor directions.
- The four built-in encoders provide motor-velocity feedback only. Their positions are not used to estimate pod azimuth, and the OpMode does not reset their counts.

The motors run in `RUN_USING_ENCODER` with velocity PIDF and `BRAKE` zero-power behavior. `setVelocity()` and motor velocity telemetry use ticks/second. The calculated no-load maximum is `1150 * 145.1 / 60 = 2781.08 ticks/s`.

Each motor's velocity contains both wheel-drive and steering components. Equal motor velocities produce wheel drive; their difference produces steering. Motor feedback is therefore not an independent wheel-only sensor.

## Melonbotics Pod Encoders

Each Melonbotics Through Bore Encoder supplies quadrature on its 4-pin JST and absolute analog position on its 3-pin JST. Encoder shaft to pod-azimuth gearing is **1:1**. Clockwise means looking down from above the robot.

### Expansion Hub Quadrature

| Encoder Port | Configuration Name | Measurement | Current Software Sign |
| --- | --- | --- | --- |
| 0 | `encoderleft` | Left pod azimuth | +1, provisional |
| 1 | `encoderright` | Right pod azimuth | +1, provisional |

These connections are unchanged. Melonbotics publishes **1024 CPR**. Whether that means decoded counts or quadrature cycles is ambiguous for raw hub counting, so `COUNTS_PER_REVOLUTION = 1024.0` is an explicitly **provisional** interpretation. If it denotes cycles with four-edge decoding, the hub could instead report 4096 counts. Do not assume either interpretation: measure signed raw hub count change over a complete pod revolution before enabling drive. A 90-degree measurement helps check scale and direction but is not a substitute for a full revolution. Old REV/8192 assumptions and previous REV measurements do not calibrate these encoders.

Rolling the drive wheel without rotating the pod should not change these counts.

In Robot Configuration, assign `encoderleft` and `encoderright` to Expansion Hub **motor channels 0 and 1**. The SDK exposes quadrature encoder inputs through those channels even with no motors attached. These channel handles are read-only in both OpModes: no output, mode, direction, or hardware-reset commands are sent to them.

Both OpModes read raw controller counts, bypassing motor-channel direction adjustments. The drive checks that the encoders share a controller on ports 0/1 and are on a different controller from the drive motors. Confirm the actual hub assignment in Robot Configuration; names alone do not establish physical wiring.

### Control Hub Analog

| Analog Channel | Configuration Name | Measurement |
| --- | --- | --- |
| 0 | `absenc` | Left pod absolute azimuth |
| 1 | `absenc2` | Right pod absolute azimuth |

Use **ONE joiner cable into the Control Hub's physical analog connector labeled 0-1**. The joiner routes two independent encoder signals to channels 0 and 1; it does not merge them into a single input. Configure **two AnalogInput hardware names** as above. Confirm the actual assignment by moving one pod at a time and checking connection telemetry; names alone do not prove wiring. Follow the manufacturer's joiner wiring and REV port pinout rather than tying the two signal wires together.

Melonbotics specifies a **0..3.2 V output at 3.3 V supply**, 12-bit DAC, maximum integral non-linearity of +/-1 degree, and 5 us propagation delay. The code uses `rawDegrees = volts / 3.2 * 360`, not the hub's maximum input voltage. Its voltage check accepts finite values in the inclusive range 0..3.2 V. **0 V can be a valid angle or a disconnected sensor**; this check is not reliable disconnect protection.

### Forward References and Startup

Calibration and tracking are shared by both OpModes in [SwervePodEncoder.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SwervePodEncoder.java). `LEFT_FORWARD_DEGREES` and `RIGHT_FORWARD_DEGREES` are independent raw analog references for the **directed wheel-travel axis facing robot-forward**, not merely a wheel plane that could point forward or backward. Both references are intentionally `NaN`, and `CALIBRATION_VERIFIED` is intentionally `false`. The drive commands zero velocity during setup and refuses powered operation until calibration is entered and verified; pressing Start while uncalibrated exits rather than driving.

The right module is rotated 180 degrees, **not mirrored**. Its independent forward reference absorbs that mounting rotation: do not add another 180 degrees or negate right feedback just because it is rotated. Determine analog and quadrature signs from actual movement; all four signs currently default to +1. Both pods remain centered at the existing left/right positions, so the kinematics and motor mixing are unchanged. Physically verify the directed positive wheel-drive and steering directions on both modules.

Once calibrated, keep the pods stationary at Start. The drive obtains fresh hub snapshots **after INIT**, reads both analog voltages and matching quadrature counts, then seeds each tracker and initial steering target from its measured absolute angle. Movement during INIT is included; even small offsets from calibrated forward are preserved rather than declared zero. There is no need to manually align forward at every Start.

```text
initial angle = wrap((raw analog degrees - pod forward degrees) * analog sign)
delta angle (radians) = delta encoder counts * quadrature sign * 2 * PI / COUNTS_PER_REVOLUTION
```

After seeding, the drive uses **quadrature only**, with no runtime analog correction or reseeding. Counter differences are computed as integers to handle signed rollover, and accumulated azimuth is wrapped. Pod angular rate is calculated from those differences and elapsed loop time. The motor-encoder gearing calculation from earlier versions is no longer used for steering feedback.

### Mechanical Analog Centering

Mechanically index each encoder so directed robot-forward is near **1.6 V = 180 raw degrees** where practical. This gives a +/-100-degree operating excursion about forward with about 80 degrees remaining to the DAC wrap on either side, or a +/-80-degree excursion with about 100 degrees remaining. Measure each actual forward reference rather than assuming exactly 180 degrees, even if both encoders are centered.

A software forward offset changes reported robot-relative zero; it **cannot move the hardware voltage wrap**. The 3.2-to-0 V transition is an angular wrap, not a dead zone or hard steering limit. These margins describe distance to that transition, not enforced travel constraints. Shortest-path steering bounds target error, not physical azimuth or overshoot.

### Sources

- [Melonbotics Through Bore Encoder specifications](https://docs.melonbotics.com/through-bore-encoder.md): published 1024 CPR and output connectors.
- [Melonbotics analog output guide](https://docs.melonbotics.com/through-bore-encoder/how-to-use-analog-output.md): 0..3.2 V range, analog specifications, and two-encoder joiner wiring.
- [REV Control/Expansion Hub port pinouts](https://docs.revrobotics.com/duo-control/control-system-overview/port-pinouts.md): physical connector/channel wiring.

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

1. Run the existing **Swerve Pod Encoder Test**, enhanced for analog and quadrature readings during both INIT and run. It is read-only: no motor outputs, mode/direction changes, or hardware encoder resets. Move one pod at a time to verify analog and quadrature channel assignment.
2. Align both directed wheel-travel axes robot-forward. Tap **A** with both analog readings valid to capture both forward voltages/raw degrees and zero displayed quadrature deltas/tracker angles in software. The initial count baseline is established automatically on the first valid hub sample, but forward voltages are captured only with A. No calibration is persisted or automatically transferred to the drive; this is intentionally a report-back workflow.
3. Record both captured forward voltages/raw degrees and wrap margins. The test shows current raw voltage/angle, raw counts, counts from zero, provisional quadrature degrees, and CW/CCW distance to the DAC wrap using provisional signs. After A it also shows captured minimum wrap margin, analog angle from captured forward, quadrature-tracked angle, and their wrapped difference.
4. From the capture position, manually rotate each pod 90 degrees clockwise viewed from above and record signed raw count delta and whether analog angle increases or decreases, accounting for wrap. Continue to one complete revolution from the capture position and record the signed count delta again without recapturing A. Use raw counts, not provisional derived degrees, to establish counts/revolution. Confirm wheel rolling alone does not change pod counts.
5. **Send back both forward voltages/raw degrees, each pod's signed count deltas at CW 90 degrees and a full revolution, analog direction, and verified channel assignment.** Leave powered drive blocked until the measurements are reviewed and forward references, signs, and counts/revolution are set and verified in the shared constants. Restarting the diagnostic loses its captures by design.
6. For subsequent powered commissioning, securely raise the wheels, reduce drive/steering limits, and keep clear of moving pods. Physically verify each module's directed positive wheel-forward and clockwise steering response; the rotated right module's reference is not a substitute for this check. INIT commands zero velocity rather than disabling motors for free manual alignment. Keep pods stationary at Start for the analog/quadrature seed.
7. Test forward/reverse, strafing, clockwise/counterclockwise turning, combined motion, and turn-stick release at low speed. Test pod-angle wraparound, shortest-path reversals near 90 degrees, and Stop behavior. Tune velocity control, steering PD, slew, and driver feel before increasing limits.

The drive reads one validated bulk snapshot per configured hub per loop and updates telemetry at 10 Hz. A detected invalid snapshot or excessive loop delay stops driving and requires a restart. Shutdown attempts zero velocity on every drive motor, even if another stop command throws, and restores prior hub caching modes. The encoder diagnostic also temporarily uses manual caching and restores prior modes at Stop.

Software checks cannot interrupt a blocked SDK call or guarantee delivery of a failed stop command. A disconnected or frozen quadrature encoder can still return valid hub data and is not reliably detected. A disconnected analog input can read a valid 0 V and seed an incorrect angle; there is no reliable disconnect protection, and analog is not used for runtime correction. Because every configured hub is checked, an unrelated hub fault can also stop the drive or suppress a diagnostic reading.

The debug APK build and 15 shared-tracker unit tests passed with `./gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug`. These tests cover encoder math, not real hub wiring or powered OpMode behavior. Physical calibration, PIDF/PD tuning, loaded speed, stopping behavior, and actual Control Hub loop timing remain to be verified on the robot. No Pinpoint commissioning is needed for this setup.
