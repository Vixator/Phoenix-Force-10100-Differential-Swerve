# Drivetrain Hardware

This document describes the current [Differential Swerve TeleOp](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DifferentialSwerveTeleOp.java), [combined pod encoder diagnostic](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SwervePodEncoderTest.java), [analog encoder display](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PodAnalogEncoderTest.java), and [individual motor drive diagnostic](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MotorEncoderDriveTest.java). See [README.md](README.md) for the operating overview, [PROJECT_DESIGN.md](PROJECT_DESIGN.md) for architecture and coding requirements, and [HUMAN_TASKS.md](HUMAN_TASKS.md) for the complete human-run calibration checklist.

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
- Built-in encoder resolution: **28 pulses per encoder-shaft revolution** and **145.1 pulses per gearbox-output revolution**; 28 at the encoder shaft is not the output-shaft value.
- Each motor encoder connects to the encoder port matching its motor port on the Control Hub.
- All four motors are wired red-to-red/black-to-black and use software direction `FORWARD`.
- Confirmed individual motor behavior with the robot raised, using `Individual Motor Drive Test` and joystick-up/positive command: `motor0` and `motor2` rotate their pods counterclockwise; `motor1` and `motor3` rotate their pods clockwise, viewed from above. Joystick-down reverses each response. All four software directions remain `FORWARD`.
- Initial combined testing showed positive differential steering turned both pods left/counterclockwise. The selected-pod diagnostic converts right-stick-right to negative differential steering; startup and runtime closed-loop steering use the same conversion. Main-drive right-stick X commands chassis rotation and stays clockwise-positive. With both pods facing forward according to their analog-zero references, equal positive wheel commands propel the robot forward; this was confirmed during powered testing.
- The four built-in encoders provide motor-velocity and raw count feedback. The active drive OpMode does not use their positions to estimate pod azimuth and does not reset their counts. `Individual Motor Drive Test` powers one selected motor at a time, capped at 25%, or drives one selected pod with the real differential mix at a capped 420 ticks/s. Hold LB for the left pod or RB for the right pod; left stick is drive and right stick X is steering. Use it with the robot raised.

The motors run in `RUN_USING_ENCODER` with velocity PIDF and `BRAKE` zero-power behavior. The physical hardware configuration, separate quadrature hub, and documented differential assembly have been verified. `setVelocity()` and motor velocity telemetry use ticks/second. The calculated no-load maximum is `1150 * 145.1 / 60 = 2781.08 ticks/s`.

Each motor's velocity contains both wheel-drive and steering components. Equal motor velocities produce wheel drive; their difference produces steering. Motor feedback is therefore not an independent wheel-only sensor.

## Melonbotics Pod Encoders

Each Melonbotics Through Bore Encoder supplies quadrature on its 4-pin JST and absolute analog position on its 3-pin JST. Encoder shaft to pod-azimuth gearing is **1:1**. Clockwise means looking down from above the robot.

### Expansion Hub Quadrature

| Encoder Port | Configuration Name | Measurement | Current Software Sign |
| --- | --- | --- | --- |
| 0 | `encoderleft` | Left pod azimuth | +1, confirmed clockwise-positive |
| 1 | `encoderright` | Right pod azimuth | +1, confirmed clockwise-positive |

These connections are unchanged. Melonbotics specifies **1024 CPR quadrature output resolution** on the 4-pin JST. The encoder body also provides absolute analog output on the 3-pin JST. Both outputs measure the same shaft, and the encoder shaft is mechanically **1:1 with pod azimuth**. The measured result is approximately 4100 raw counts per pod revolution on both pods. This is close enough to the configured `COUNTS_PER_REVOLUTION = 4096.0` and is mechanically consistent with the Melonbotics 1024 CPR encoder using 4x quadrature decoding. A 90-degree measurement helps check scale and direction, but is not a substitute for a full revolution.

Rolling the drive wheel without rotating the pod should not change these counts.

In Robot Configuration, assign `encoderleft` and `encoderright` to Expansion Hub **motor channels 0 and 1**. The SDK exposes quadrature encoder inputs through those channels even with no motors attached. These channel handles are read-only for quadrature measurement: no output or hardware-reset command is sent to them. The encoder diagnostic’s explicit Y alignment action commands only the selected drive pod’s motors; it never commands the quadrature channel handles.

Both OpModes read raw controller counts, bypassing motor-channel direction adjustments. Their normal measurement paths are read-only; the documented alignment hotkeys are the only motor-command exception. The drive checks that the encoders share a controller on ports 0/1 and are on a different controller from the drive motors. Confirm the actual hub assignment in Robot Configuration; names alone do not establish physical wiring.

### Control Hub Analog

| Analog Channel | Configuration Name | Measurement |
| --- | --- | --- |
| 0 | `absencleft` | Left pod absolute azimuth |
| 1 | `absencright` | Right pod absolute azimuth |

Use **ONE joiner cable into the Control Hub's physical analog connector labeled 0-1**. The joiner routes two independent encoder signals to channels 0 and 1; it does not merge them into a single input. Configure **two AnalogInput hardware names** as above. Confirm the actual assignment by moving one pod at a time and checking connection telemetry; names alone do not prove wiring. Follow the manufacturer's joiner wiring and REV port pinout rather than tying the two signal wires together.

Melonbotics specifies a **0..3.2 V output at 3.3 V supply**, 12-bit DAC, maximum integral non-linearity of +/-1 degree, and 5 us propagation delay. The code uses `rawDegrees = min(volts, 3.2) / 3.2 * 360`, not the hub's maximum input voltage. Validation accepts finite 0..3.3 V readings; the upper 0.1 V margin is clamped to the wrap endpoint to avoid rejecting slightly overscale measurements. Invalid feedback reports the measured voltage. **0 V can be a valid angle or a disconnected sensor**; this check is not reliable disconnect protection.

### Forward References and Startup

Calibration and tracking are shared by the drive and encoder diagnostics through [SwervePodEncoder.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SwervePodEncoder.java). The complete physical discovery procedure is recorded in [HUMAN_TASKS.md](HUMAN_TASKS.md). `LEFT_FORWARD_DEGREES` and `RIGHT_FORWARD_DEGREES` are independent raw analog references for the **directed wheel-travel axis facing robot-forward**, not merely a wheel plane that could point forward or backward. The measured top-dead-center zero references are left **0.122 V = 13.725°** and right **0.258 V = 29.025°**. `CALIBRATION_VERIFIED` is now `true` because hardware configuration, encoder signs/scale, individual motor responses, and combined pod steering direction have been verified. The drive still performs bounded startup alignment and refuses to continue if alignment or feedback fails.

The right module is rotated 180 degrees, **not mirrored**. Its independent forward reference absorbs that mounting rotation: do not add another 180 degrees or negate right feedback just because it is rotated. The analog signs are confirmed as `-1` for both pods because both raw voltages decrease during clockwise rotation. Both quadrature signs are confirmed as `+1` because clockwise rotation increased both raw counts. The accepted software scale is 4096 counts per revolution; both pods measured approximately 4100 counts, consistent with the 1024 CPR / 4x quadrature specification. Both pods remain centered at the existing left/right positions, so the kinematics and motor mixing are unchanged. Combined pod steering direction and forward wheel-drive direction have been confirmed; loaded behavior remains a commissioning task.

Once calibrated, the drive obtains fresh hub snapshots **during INIT** and automatically steers both pods to their independent analog forward references. Alignment uses proportional steering capped at 0.20 command, with a 0.03 minimum command outside the 2° settling window to overcome static friction near the target. It requires 100 ms continuously inside the 2° target window and times out after 5 seconds. Either pod failing stops both. READY indicates completion; motors wait at zero velocity until Start. An early Start waits for alignment to finish. At Start, a fresh validated snapshot supplies quadrature baselines and residual analog angles. Both pods must still be within tolerance; if moved while waiting, reinitialize. **Pods move on INIT; keep clear before pressing INIT.**

```text
initial angle = wrap((raw analog degrees - pod forward degrees) * analog sign)
delta angle (radians) = delta encoder counts * quadrature sign * 2 * PI / COUNTS_PER_REVOLUTION
```

After seeding, the drive uses **quadrature only**, with no runtime analog correction or reseeding. Counter differences are computed as integers to handle signed rollover, and accumulated azimuth is wrapped. Pod angular rate is calculated from those differences and elapsed loop time. The motor-encoder gearing calculation from earlier versions is no longer used for steering feedback.

### Mechanical Analog Centering

Mechanically index each encoder so directed robot-forward is near **1.6 V = 180 raw degrees** where practical. This gives a +/-100-degree operating excursion about forward with about 80 degrees remaining to the DAC wrap on either side, or a +/-80-degree excursion with about 100 degrees remaining. Measure each actual forward reference rather than assuming exactly 180 degrees, even if both encoders are centered.

A software forward offset changes reported robot-relative zero; it **cannot move the hardware voltage wrap**. The 3.2-to-0 V transition is an angular wrap, not a dead zone or hard steering limit. These margins describe distance to that transition, not enforced travel constraints. Shortest-path steering bounds target error, not physical azimuth or overshoot.

### Sources

- [Melonbotics Through Bore Encoder specifications](https://docs.melonbotics.com/through-bore-encoder.md): 7 mm hex bore compatible with 8 mm REX, 1024 CPR quadrature output, and analog output.
- [Melonbotics analog output guide](https://docs.melonbotics.com/through-bore-encoder/how-to-use-analog-output.md): 0..3.2 V range, analog specifications, and two-encoder joiner wiring.
- [REV Control/Expansion Hub port pinouts](https://docs.revrobotics.com/duo-control/control-system-overview/port-pinouts.md): physical connector/channel wiring.

## Mechanical Geometry and Programmed Ratio

| Parameter | Confirmed Value |
| --- | --- |
| Wheel diameter | 63.25 mm |
| Pod wheel-center spacing | 359.5 mm |
| Pod locations | Directly left/right of robot center |
| Initial bevel pair, each motor | 1:1 |
| Spur stage | 16-tooth driving 54-tooth |
| Wheel bevel stage | 50-tooth driving 19-tooth |
| Wheel-drive ratio | `(16/54) * (50/19) = 0.779727` |

Each motor drives a horizontal-to-vertical 1:1 bevel pair followed by its own 16:54 stage. The right motor feeds the upper differential part; the left motor feeds the lower part. The upper and lower 54-tooth gears are rigidly attached to their respective 50-tooth bevel gears, and both 50-tooth gears engage the common 19-tooth wheel gear. Equal motor motion is the wheel-drive component; motor motion in opposite directions is the pod-steering component. This is the known differential topology used by the motor-mixing code.

The complete programmed external ratio is `(16/54) * (50/19) = 0.779727` wheel revolutions per motor-output revolution. With the 1150 RPM motor rating and 63.25 mm wheel, the theoretical no-load wheel speed is approximately **2.970 m/s**. This is used to convert chassis rotation requests into normalized pod speeds; it is not a measured loaded operating limit.

## Controls and Motor Mixing

- Left-stick direction selects robot-relative travel direction; distance from center selects speed after radial deadband rescaling.
- Right-stick X commands rotation about the pod midpoint: right is clockwise, left is counterclockwise. Rotation input is cubed after its deadband.
- Full-stick pure rotation uses the full wheel-speed range of both pods. When aligned forward, all four hardware motors request approximately +2781 ticks/s for a right turn; a left turn reverses all four signs. Logical wheel speeds are opposite: left forward/right reverse for clockwise motion, followed by the right pod negate-and-swap hardware mapping. Misalignment reduces drive to preserve steering authority.
- Releasing right-stick X commands zero chassis rotation, with no heading hold or return-to-forward behavior.
- Triggers are unused.
- At zero requested pod velocity, its previous azimuth target is retained. A wholly zero chassis command sets all four motor velocities to zero with BRAKE; a zero-speed pod within a nonzero chassis request may still steer toward its retained target.

Per pod, the controller optimizes the requested vector by reversing wheel direction when appropriate. A 3° hysteresis band around the 90° reversal boundary prevents chatter, allowing up to 93° retained steering error. It applies PD steering with measured-rate damping, a steering-command slew limit, and cosine-squared alignment scaling; wheel drive is zero at or beyond 90° error.

```text
left motor velocity  = wheel-drive component + steering component
right motor velocity = wheel-drive component - steering component
```

Positive steering in this motor mix is **counterclockwise** on both measured pods. The clockwise-positive PD correction is converted at the motor boundary: `steering component = -(STEERING_KP * error - STEERING_KD * measuredRate)`. Startup alignment and runtime steering share this conversion. The previous runtime implementation omitted it, creating positive feedback as soon as motion introduced an angle error. Main-drive chassis rotation must not be inverted to compensate for this pod-level polarity.

Steering has priority within the motor-speed limit; drive uses the remaining headroom. This avoids independently clipping the motor commands and changing the requested steering component. Driving and steering cannot simultaneously demand each motor's full capacity.

## Preliminary Tuning

These are current source defaults, **not validated gains or safe operating limits**:

| Setting | Value | Meaning |
| --- | --- | --- |
| Pod `DEFAULT_KP` | 0.5 | Normalized steering output per radian of error |
| Pod `DEFAULT_KD` | 0.01 | Measured-rate damping, normalized output per rad/s |
| `PedroDriveConfig.TELEOP_MAX_DRIVE` | 1.0 | Wheel-drive command scale |
| Pod `DEFAULT_MAX_STEER` | 0.20 | Steering component limit, matching startup cap |
| `DRIVE_DEADBAND`, `TURN_DEADBAND` | 0.05 | Stick deadbands |
| Input `MAX_TURN_RATE` | `2 * MAX_WHEEL_SPEED_METERS_PER_SECOND / TRACK_WIDTH_METERS`, about 16.52 rad/s | Theoretical full-stick pure rotation scale: opposite wheels at full speed, not a measured loaded chassis rate |
| Pod `DEFAULT_SLEW_RATE` | 2.0/s | Normalized steering-command change limit; nonpositive disables it |
| `SwerveTuning.MOTOR_VELOCITY_P/I/D` | 15.0, 0.5, 0.5 | REV motor velocity controller gains |
| `SwerveTuning.MOTOR_VELOCITY_F` | `32767 / 2781.0833`, about 11.782 | Preliminary REV velocity feedforward, not volts/RPM |

`SwerveTuning` owns live motor PIDF, steering PD/slew/cap, and alignment tuning. `SwerveDriverInput` owns input shaping; `PedroDriveConfig` owns drive profiles. The runtime reapplies changed PIDF values, and the pods validate output limits on every preparation. Record tuned values back into source before redeploying; Dashboard changes are session-only.

## Commissioning and Faults

1. Run the **Pod Analog Encoder Test**. LB selects the left pod, RB selects the right pod, A steers the selected pod toward its configured forward analog reference, and B aborts. If a reference has not yet been entered, the test targets analog 0 degrees. Keep the robot raised and clear during alignment.
2. Run the **Individual Motor Drive Test** with the robot raised. D-pad selects `motor0`–`motor3` for individual direction checks. Hold LB to drive the left pod or RB to drive the right pod with the actual `drive + steer` / `drive - steer` differential mix; left stick is drive, right stick X is steering, and combined output is capped at 420 ticks/s.
3. Run the **Swerve Pod Encoder Test**. LB/RB select a pod, A zeroes the selected quadrature display in software, X starts/finishes a manual 360-degree measurement, Y steers the selected pod to its analog forward reference, and B aborts alignment.
4. Verify the accepted 4096-count scale and positive clockwise sign. Wheel rotation has been confirmed not to change pod quadrature counts.
5. The analog zero references are recorded as left 0.122 V / 13.725° and right 0.258 V / 29.025°. Encoder scale/sign, analog polarity, channel assignment, and combined pod steering direction are confirmed; `CALIBRATION_VERIFIED` is enabled. Continue with powered drive commissioning.
6. Run the main drive with wheels raised and reduced limits. INIT aligns and settles both pods. Wait for READY, then press Start to capture fresh quadrature baselines and drive. Alignment timeout or invalid feedback stops both pods.
8. Test forward/reverse, strafing, clockwise/counterclockwise turning, combined motion, and turn-stick release at low speed. Test pod-angle wraparound, shortest-path reversals near 90 degrees, and Stop behavior. Tune velocity control, steering PD, slew, and driver feel before increasing limits.

The drive reads one validated bulk snapshot per configured hub per normal loop and updates telemetry at 10 Hz. An invalid read commands zero velocity before retrying the entire hub set: up to three total attempts within 150 ms, 10 ms apart. During driving, recovered reads require neutral sticks and analog/quadrature agreement within 10° before resuming; trackers are not reseeded. Recovery time counts toward the 250 ms loop limit. Persistent failures latch a stopped telemetry state until Stop/reinitialization. Fault telemetry/logs include the configured hub name, connection, parent/downstream role, and failure reason; a module address alone does not identify Control versus Expansion Hub. Shutdown attempts every motor stop and restores prior hub caching modes. The encoder diagnostic also temporarily uses manual caching and restores prior modes at Stop.

Software checks cannot interrupt a blocked SDK call or guarantee delivery of a failed stop command. A disconnected or frozen quadrature encoder can still return valid hub data and is not reliably detected. A disconnected analog input can read a valid 0 V and seed an incorrect angle; there is no reliable disconnect protection, and analog is not used for runtime correction. Because every configured hub is checked, an unrelated hub fault can also stop the drive or suppress a diagnostic reading.

The debug APK build and unit tests passed with `./gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug`. Tests cover encoder math, steering/damping polarity, chassis kinematics, limits, and simulated closed-loop convergence. They do not exercise real hub wiring or powered OpMode behavior. Complete [HUMAN_TASKS.md](HUMAN_TASKS.md) for powered commissioning, tuning, and final acceptance. PIDF/PD tuning, loaded speed, stopping behavior, actual Control Hub loop timing, and Pinpoint frame verification remain to be completed on the robot.

## Pedro / Pinpoint integration settings

The application consumes `com.pedropathing:revhub:3.0.1`. The robot frame is X forward, Y left,
and CCW-positive yaw. Drive-pod centers are `(0, +7.0767716535 in)` left and
`(0, -7.0767716535 in)` right. The Pinpoint odometry offsets are independent: X pod
`+199.25 mm` (left of center) and Y pod `+88.0 mm` (forward of center), using two
`goBILDA_4_BAR_POD` presets and global inches.

Current commissioned direction settings are X `FORWARD`, Y `FORWARD`, with a
`COUNTERCLOCKWISE_POSITIVE` heading convention. `PinpointSettings` accepts these values for powered
Pedro arming. Continue monitoring the following during commissioning:

| Observation | Expected | Recorded result |
| --- | --- | --- |
| Forward translation | field X increases | PASS |
| Left translation | field Y increases | PASS |
| CCW rotation | heading and angular rate increase | PASS |
| Forward at +90 degrees | field Y increases | PASS |
| 24 in forward scale | within 2% | PASS |
| 24 in left scale | within 2% | PASS |
| Rotation-in-place translation drift | at most 1 in initially | PASS |

Autonomous commissioning uses a `0.15` wheel-drive component and `0.20` normalized turn envelope.
The existing `0.20` steering component is retained, for a configured combined commissioning cap of
`0.35`. Motor targets remain: left pod `(logicalLeft, logicalRight)` and right pod
`(-logicalRight, -logicalLeft)`, multiplied once by `2781.083333 ticks/s`. Zero commands and all
faults retain `RUN_USING_ENCODER` plus BRAKE; voltage compensation and X-lock are disabled.

The 0.35 autonomous combined cap is enforced at each motor preparation, including after Dashboard edits; increasing steering above the available commissioning headroom stops the run. Normal TeleOp uses a separate 1.0 motor envelope. Hardware loop timing, actual stop delivery, and loaded dynamics remain subject to physical testing. See [INTEGRATION_STATUS.md](INTEGRATION_STATUS.md) for the September 21 software verification results.
