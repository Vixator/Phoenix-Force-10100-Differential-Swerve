# Human Drivetrain Calibration and Test Tasks

This checklist contains the work that software cannot perform. Complete it on the actual robot and record the results here before enabling powered drive.

Related documentation:

- [README.md](README.md) — operating overview and controls
- [hardware.md](hardware.md) — wiring, specifications, ratios, and commissioning details
- [PROJECT_DESIGN.md](PROJECT_DESIGN.md) — architecture, safety, and design decisions
- [SwervePodEncoder.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SwervePodEncoder.java) — calibration constants to update after review

## Current software gate

Powered `Differential Swerve TeleOp` calibration is enabled after the confirmed encoder and combined-steering checks:

```java
CALIBRATION_VERIFIED = true
LEFT_FORWARD_DEGREES = 13.725   // 0.122 V top-dead-center
RIGHT_FORWARD_DEGREES = 29.025  // 0.258 V top-dead-center
```

Calibration is enabled. Complete the powered commissioning and tuning tasks below before normal operation.

## Safety rules

- Securely raise the robot for all initial motor tests so the wheels cannot contact the floor.
- Keep hands, clothing, cables, and tools clear of the pods and gears.
- Have an operator at the Driver Station and a second person at the robot whenever motors may move.
- Keep the physical battery disconnect or emergency stop accessible.
- Use reduced calibration velocity and stop immediately if a pod moves in the wrong direction, stalls, chatters, or approaches a mechanical limit.
- Use **B** to abort the analog-assisted alignment actions.
- Never assume that a valid 0 V reading proves an analog encoder is connected; 0 V can be a valid encoder position.
- Stop and correct wiring or software signs if a pod moves opposite the expected direction.
- Record results only after the pod has stopped and the reading is stable.

## A. Hardware identity and wiring

### A1. Drive motor identity

- [x] Confirm all four motors are goBILDA `5203-2402-0005`.
- [x] Confirm each motor has the documented 5.2:1 planetary gearbox.
- [x] Confirm the motor encoder wiring follows the matching Control Hub motor port.
- [x] Confirm the physical port/name mapping:

| Port | Program name | Physical location | Confirmed |
|---:|---|---|:---:|
| 0 | `motor0` | Left pod, left motor | [x] |
| 1 | `motor1` | Left pod, right motor | [x] |
| 2 | `motor2` | Right pod, left motor | [x] |
| 3 | `motor3` | Right pod, right motor | [x] |

### A2. Pod quadrature wiring

- [x] Confirm the left Melonbotics encoder quadrature output is on Expansion Hub motor channel 0 and named `encoderleft`.
- [x] Confirm the right Melonbotics encoder quadrature output is on Expansion Hub motor channel 1 and named `encoderright`.
- [x] Confirm both quadrature channels are on the same Expansion Hub.
- [x] Confirm the quadrature hub is separate from the Control Hub carrying the four drive motors.
- [x] Confirm no motor is required to be attached to those Expansion Hub motor channels.
- [x] Confirm the encoder connectors and strain relief cannot contact moving gears.

### A3. Pod analog wiring

- [x] Confirm one joiner cable is connected to the Control Hub physical analog connector labeled `0-1`.
- [x] Confirm the left analog signal is channel 0 and named `absencleft`.
- [x] Confirm the right analog signal is channel 1 and named `absencright`.
- [x] Move only the left pod and confirm only `absencleft` changes.
- [x] Move only the right pod and confirm only `absencright` changes.
- [x] Confirm the two signals are not electrically tied together.
- [x] Record connection information shown by the diagnostic tools:

```text
Analog joiner/cable notes: verified
Left analog connection: verified
Right analog connection: verified
Quadrature hub connection: verified
```

## B. Mechanical configuration

### B1. Differential gear assembly

Confirm the installed assembly matches the programmed model:

```text
Each motor: 1:1 bevel pair → 16-tooth → 54-tooth
Each 54-tooth gear is fixed to a 50-tooth gear
Both 50-tooth gears engage the common 19-tooth wheel gear
Left motor: lower differential path
Right motor: upper differential path
```

- [x] Confirm the initial bevel pair for each motor is 1:1.
- [x] Confirm four 16-tooth gears are installed as the driving gears.
- [x] Confirm four 54-tooth gears are installed as the differential gears.
- [x] Confirm each 54-tooth gear is rigidly attached to its 50-tooth gear.
- [x] Confirm the two 50-tooth gears engage the common 19-tooth wheel gear.
- [x] Confirm the right pod is rotated 180° mechanically but is not mirrored.
- [x] Confirm the gears rotate freely without binding through the full allowed pod range.
- [x] Confirm all gear set screws, hubs, shims, and structural fasteners are secure.

### B2. Geometry

- [x] Measure and record wheel diameter:

```text
Measured wheel diameter: 63.25 mm
Code value currently used: 63.25 mm
```

- [x] Measure and record pod wheel-center spacing:

```text
Measured pod spacing: 359.5 mm
Code value currently used: 359.5 mm
```

- [x] Confirm both pods are centered at the documented left/right locations.
- [x] Confirm the installed wheel/tread type and document any difference from the BOM. Installed tread: 40A belt tires from Melonbotics.
- [x] Confirm the wheel does not slip on its 8 mm REX/compatible mounting. Slip occurs only when pushing against an immovable object; additional robot weight is expected to mitigate it.

## C. Motor direction and differential behavior

These checks require the robot to be raised. Use low calibration speed.

### C1. Individual motor sanity

Use the appropriate motor diagnostic procedure and verify that each motor’s physical direction is understood before running the main drive.

| Motor | Expected physical response | Result/notes |
|---|---|---|
| `motor0` | Left pod lower-path motor direction known | joystick up: pod counterclockwise; joystick down: pod clockwise | |
| `motor1` | Left pod upper-path motor direction known | joystick up: pod clockwise; joystick down: pod counterclockwise | |
| `motor2` | Right pod lower-path motor direction known | joystick up: pod counterclockwise; joystick down: pod clockwise | |
| `motor3` | Right pod upper-path motor direction known | joystick up: pod clockwise; joystick down: pod counterclockwise | |

- [x] Confirm all four programmed software directions remain `FORWARD`.
- [x] Record individual motor pod-response directions below.
- [ ] Confirm equal positive commands propel the left pod in its directed robot-forward wheel-travel direction.
- [ ] Confirm equal positive commands propel the right pod in its directed robot-forward wheel-travel direction.
- [x] Confirm the corrected driver right-stick input steers each pod clockwise/right.
- [x] Confirm right-stick left steers each pod counterclockwise/left.
- [ ] Confirm the right pod’s 180° mounting does not require an additional software mirror or 180° correction.

Record any correction required before proceeding:

```text
Left pod motor sign correction needed:  none / yes: __________
Right pod motor sign correction needed: none / yes: __________
```

## D. Analog forward-reference discovery

Run `Pod Analog Encoder Test`.

Controls:

- **LB** selects left pod.
- **RB** selects right pod.
- **A** commands the selected pod toward its configured target.
- **B** aborts motor-assisted alignment.

Until source constants are populated, the diagnostic targets analog 0°. For initial discovery, manually position each directed wheel-travel axis robot-forward and record the stable displayed analog value. If needed, use the bounded A alignment action only after confirming motor steering direction.

### D1. Left forward reference

- [ ] Select the left pod with LB.
- [ ] Align the left directed wheel-travel axis with robot-forward.
- [ ] Record the stable analog voltage:

```text
Left forward voltage: 0.122 V
```

- [ ] Record the stable raw analog angle shown by telemetry:

```text
Left forward raw degrees: 13.725 °
```

### D2. Right forward reference

- [ ] Select the right pod with RB.
- [ ] Align the right directed wheel-travel axis with robot-forward.
- [ ] Record the stable analog voltage:

```text
Right forward voltage: 0.258 V
```

- [ ] Record the stable raw analog angle shown by telemetry:

```text
Right forward raw degrees: 29.025 °
```

### D3. Analog direction and repeatability

For each pod, rotate slowly clockwise viewed from above and observe whether raw analog angle increases or decreases. Account for wraparound.

| Pod | Clockwise analog behavior | Programmed analog sign | Confirmed |
|---|---|---:|:---:|
| Left | decreases | `-1` | [x] |
| Right | decreases | `-1` | [x] |

- [x] Repeat the forward alignment and confirm it returns extremely close to top dead center for both pods.
- [x] Analog repeatability is accepted as good enough for this application.

```text
Left forward reference spread: very small / accepted
Right forward reference spread: very small / accepted
```

- [x] Confirm the forward reference is acceptable for the intended steering range.

## E. Quadrature channel, sign, and resolution discovery

Run `Swerve Pod Encoder Test`.

Controls:

- **LB/RB** select the left/right pod.
- **A** zeroes the selected quadrature display in software only.
- **X** starts and finishes the full-revolution measurement.
- **Y** steers the selected pod to its configured analog reference.
- **B** aborts alignment.

### E1. Left quadrature

- [ ] Select the left pod.
- [ ] Press A to capture the left software zero.
- [ ] Rotate the left pod exactly 90° clockwise and record signed count delta:

```text
Left CW 90° signed count delta: ______ counts
```

- [ ] Return to the capture point without pressing A again.
- [ ] Press X, rotate the left pod exactly one complete revolution clockwise, and press X again.
- [ ] Record the signed full-revolution count:

```text
Left CW 360° signed count delta: ______ counts
```

- [ ] Confirm magnitude is approximately 4096 raw counts by physical full-revolution measurement; current software assumes the theoretical 4096-count scale.
- [x] Record the quadrature direction result:

```text
Left quadrature sign required: +1 (clockwise increased counts)
```

### E2. Right quadrature

- [ ] Select the right pod.
- [ ] Press A to capture the right software zero.
- [ ] Rotate the right pod exactly 90° clockwise and record signed count delta:

```text
Right CW 90° signed count delta: ______ counts
```

- [ ] Return to the capture point without pressing A again.
- [ ] Press X, rotate the right pod exactly one complete revolution clockwise, and press X again.
- [ ] Record the signed full-revolution count:

```text
Right CW 360° signed count delta: ______ counts
```

- [ ] Confirm magnitude is approximately 4096 raw counts by physical full-revolution measurement; current software assumes the theoretical 4096-count scale.
- [x] Record the quadrature direction result:

```text
Right quadrature sign required: +1 (clockwise increased counts)
```

### E3. Wheel-motion isolation

- [x] Confirmed that rolling the left wheel without rotating the left pod does not change the pod quadrature count.
- [x] Confirmed that rolling the right wheel without rotating the right pod does not change the pod quadrature count.
- [ ] If counts change, stop and inspect mechanical coupling or wiring before enabling drive.

## F. Individual motor direction and differential verification

Run `Individual Motor Drive Test` with the robot raised.

Controls:

```text
D-pad Up:    select motor0
D-pad Right: select motor1
D-pad Down:  select motor2
D-pad Left:  select motor3
Left stick Y: individual motor command, capped at 25% power
LB:           combined left-pod mode
RB:           combined right-pod mode
Left stick Y: combined drive command
Right stick X: combined steering command
Combined output: capped at 420 ticks/s
```

- [x] Raise the robot and confirm only the selected motor moves.
- [x] Record the physical rotation direction of each motor:

```text
motor0 direction:
motor1 direction:
motor2 direction:
motor3 direction:
```

- [ ] Confirm the left motor pair affects only the left pod.
- [ ] Confirm the right motor pair affects only the right pod.
- [ ] Hold LB and verify combined left-pod drive moves the wheel in robot-forward.
- [ ] Hold RB and verify combined right-pod drive moves the wheel in robot-forward.
- [x] Retest right-stick steering after the software polarity correction: right turns clockwise/right and left turns counterclockwise/left on both pods.
- [ ] Confirm equal motor-path motion corresponds to wheel-drive motion.
- [ ] Confirm opposite motor-path motion corresponds to pod-steering motion.
- [ ] Record any sign or assembly mismatch:

```text
Left motor/module observations:
Right motor/module observations:
Differential gear mismatch or binding:
```

## G. Enter calibration values into source

After review, update `SwervePodEncoder.java`:

```java
public static final int LEFT_QUADRATURE_SIGN = ______;
public static final int RIGHT_QUADRATURE_SIGN = ______;
public static final int LEFT_ANALOG_SIGN = -1;
public static final int RIGHT_ANALOG_SIGN = -1;
public static final double LEFT_FORWARD_DEGREES = ______;
public static final double RIGHT_FORWARD_DEGREES = ______;
public static final boolean CALIBRATION_VERIFIED = true;
```

Before setting the final flag:

- [ ] Both forward values are finite and in the range 0° through less than 360°.
- [x] Both analog signs are physically confirmed as `-1`; both voltages decrease clockwise.
- [x] Both quadrature signs are confirmed as `+1` because clockwise rotation increased counts.
- [ ] Both full-revolution magnitudes are physically confirmed as approximately 4096 raw counts using the specified power-of-two scale; current software uses the theoretical value without a pod-rotation measurement.
- [ ] Values were independently reviewed by another person.
- [ ] `README.md`, `hardware.md`, and this task file contain the same accepted values.

## H. Main drive INIT alignment

With calibration entered:

- [ ] Build and deploy the current debug APK.
- [ ] Confirm the main drive reports calibration ready.
- [ ] Place both pods away from forward so movement is observable.
- [ ] Press INIT for `Differential Swerve TeleOp` with the robot raised and clear.
- [ ] Confirm both pods automatically steer toward their independent analog forward references before pressing Start.
- [ ] Wait for READY and confirm motor velocity commands are zero while waiting for Start.
- [ ] Press Start and confirm driving begins without repeating alignment.
- [ ] In a separate run, press Start early and confirm driving waits for alignment to finish; verify Stop also cancels during INIT.
- [ ] Confirm alignment telemetry shows each pod's voltage; if feedback is rejected, record the sensor name and voltage from the fault.
- [ ] Confirm both pods stop within the configured alignment tolerance.
- [ ] Confirm the main drive refuses to proceed if a pod cannot align before timeout.
- [ ] Confirm startup requires 100 ms inside tolerance and seeds quadrature with each actual residual analog angle (near zero).
- [ ] Confirm failure of either pod stops both and a completed pod drifting out of tolerance prevents handoff.
- [ ] Restart the robot and repeat at least three times.
- [ ] Confirm the final startup pod-angle error is acceptable:

```text
Left startup error: ______ °
Right startup error: ______ °
Observed issue: one pod sometimes remains at approximately 2–2.1° and never stays below the 2° settling threshold before timeout; retrying or manually repositioning the pods can succeed.
```

## I. Low-speed powered drivetrain test

Run only after sections A–H are complete. Keep the robot raised for the first tests.

### Runtime steering polarity regression check

Reported behavior before the fix: startup analog alignment reached forward, but joystick movement triggered uncontrolled pod steering. Runtime PD used the opposite motor polarity from startup alignment. The corrected runtime loop now shares the startup polarity conversion; chassis rotation input is separately clockwise-positive. Existing measured encoder references, signs, ports, and ratios remain the calibration basis.

For the updated build, verify:

- [ ] After alignment, a small right/left strafe request moves both pods toward the requested wheel axis. `angle / target / error` telemetry should converge toward zero error rather than grow or cycle.
- [ ] Release both sticks: wheel drive becomes zero and pods settle at the retained travel axis without continuous rotation.
- [ ] With pods forward, forward/reverse input rolls the wheels while holding pod angle. Reverse should not require a 180° pod turn.
- [ ] Right-stick right requests clockwise chassis rotation: left wheel forward, right wheel backward. This differs from the diagnostic's right stick, which directly steers a selected pod.

Runtime steering is capped at 0.20 with Kp 0.5, Kd 0.01, and slew rate 2.0/s. Telemetry shows each pod's angle/target/error and both motor velocity targets. A 3° reversal hysteresis band prevents noisy wheel-direction switching near 90°. Software regression tests and the debug build pass; these physical checks remain unverified until run on the robot.

- [ ] Forward translation.
- [ ] Reverse translation.
- [ ] Robot-right strafe.
- [ ] Robot-left strafe.
- [ ] Clockwise chassis rotation.
- [ ] Counterclockwise chassis rotation.
- [ ] With translation centered and pods aligned, full right-stick rotation requests full opposite wheel speeds using both motors per pod; verify motor-target telemetry is approximately +2781/+2781 on the left and -2781/-2781 on the right for clockwise rotation. Reverse for counterclockwise.
- [ ] Translation plus clockwise rotation.
- [ ] Translation plus counterclockwise rotation.
- [ ] Release right stick and confirm zero requested rotation.
- [ ] Confirm pod targets are retained when requested pod velocity is zero.
- [ ] Confirm shortest-path steering reverses wheel direction correctly near 90°.
- [ ] Confirm analog wraparound does not cause an unexpected steering jump.
- [ ] Confirm motor velocities remain within the configured limit.
- [ ] Confirm stop behavior sets all four motor velocities to zero.
- [ ] Confirm a transient hub-read failure stops outputs before retrying, reports the recovered-read count, and requires centered sticks before runtime motion resumes.
- [ ] Confirm persistent hub feedback loss or excessive loop delay displays DRIVE STOPPED, holds zero commands, and requires Stop/reinitialization. Record configured hub name, connection, and reason from telemetry/logs.
- [ ] Confirm recovered analog/quadrature disagreement prevents resuming after a hub reset.

Record issues:

```text
Translation behavior:
Rotation behavior:
Combined-motion behavior:
Wrap/shortest-path behavior:
Stopping behavior:
Fault behavior:
```

## J. Loaded drivetrain and tuning validation

These values cannot be established from source code alone.

- [ ] Measure loaded maximum wheel speed.
- [ ] Measure chassis rotation speed.
- [ ] Tune motor velocity PIDF under load.
- [ ] Tune steering Kp.
- [ ] Tune steering Kd.
- [ ] Tune steering slew rate.
- [ ] Tune drive and steering limits for safe operation.
- [ ] Measure stopping distance and time.
- [ ] Check motor and gearbox temperature after repeated operation.
- [ ] Check battery voltage sag and current draw.
- [ ] Check gear wear, loosened fasteners, tread slip, and structural flex.
- [ ] Confirm the robot remains controllable if one pod is temporarily unloaded.
- [ ] Document final accepted tuning values in `hardware.md` and this file.

## K. Completion gate

The current drivetrain is ready for normal operation only when all are true:

- [x] Hardware names and ports match the software.
- [x] Differential gear assembly matches the documented topology.
- [x] Individual motor directions are recorded; combined pod steering signs still require confirmation.
- [x] Analog forward references are recorded: left 0.122 V / 13.725°, right 0.258 V / 29.025°.
- [x] Analog signs are recorded as `-1` for both pods.
- [x] Quadrature signs are recorded as `+1` for both pods.
- [x] Full-revolution quadrature scale is accepted as 4096 counts.
- [x] Calibration constants are entered and reviewed.
- [x] `CALIBRATION_VERIFIED` is true.
- [ ] Startup alignment succeeds repeatedly.
- [ ] Low-speed drive tests pass.
- [ ] Stop and fault behavior pass.
- [ ] Loaded speed, thermal, current, and tuning checks are documented.

## Final accepted values

Keep this section synchronized with `SwervePodEncoder.java` and `hardware.md` after commissioning.

```text
Left forward analog voltage: 0.122 V
Left forward raw degrees:    13.725 °
Right forward analog voltage: 0.258 V
Right forward raw degrees:    29.025 °
Left analog sign:             -1
Right analog sign:            -1
Left quadrature sign:         +1
Right quadrature sign:        +1
Left full-revolution counts:  assumed 4096 (theoretical; not physically measured)
Right full-revolution counts: assumed 4096 (theoretical; not physically measured)
Measured wheel diameter:      ______ mm
Measured pod spacing:         359.5 mm
Final steering KP:             ______
Final steering KD:             ______
Final motor PIDF:              ______
Final safe drive limit:        ______
Final safe steering limit:     ______
Commissioned by:              ______
Commission date:               ______
```
