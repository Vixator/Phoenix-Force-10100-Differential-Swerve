# Human Drivetrain Calibration and Test Tasks

This checklist contains the work that software cannot perform. Complete it on the actual robot and record the results here before enabling powered drive.

Related documentation:

- [README.md](README.md) — operating overview and controls
- [hardware.md](hardware.md) — wiring, specifications, ratios, and commissioning details
- [PROJECT_DESIGN.md](PROJECT_DESIGN.md) — architecture, safety, and design decisions
- [SwervePodEncoder.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SwervePodEncoder.java) — calibration constants to update after review

## Current software gate

Powered `Differential Swerve TeleOp` is intentionally blocked until all required calibration values are reviewed and entered:

```java
CALIBRATION_VERIFIED = false
LEFT_FORWARD_DEGREES = NaN
RIGHT_FORWARD_DEGREES = NaN
```

Do not set `CALIBRATION_VERIFIED = true` until the required tasks below are complete.

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

- [ ] Confirm all four motors are goBILDA `5203-2402-0005`.
- [ ] Confirm each motor has the documented 5.2:1 planetary gearbox.
- [ ] Confirm the motor encoder wiring follows the matching Control Hub motor port.
- [ ] Confirm the physical port/name mapping:

| Port | Program name | Physical location | Confirmed |
|---:|---|---|:---:|
| 0 | `motor0` | Left pod, left motor | [ ] |
| 1 | `motor1` | Left pod, right motor | [ ] |
| 2 | `motor2` | Right pod, left motor | [ ] |
| 3 | `motor3` | Right pod, right motor | [ ] |

### A2. Pod quadrature wiring

- [ ] Confirm the left Melonbotics encoder quadrature output is on Expansion Hub motor channel 0 and named `encoderleft`.
- [ ] Confirm the right Melonbotics encoder quadrature output is on Expansion Hub motor channel 1 and named `encoderright`.
- [ ] Confirm both quadrature channels are on the same Expansion Hub.
- [ ] Confirm the quadrature hub is separate from the Control Hub carrying the four drive motors.
- [ ] Confirm no motor is required to be attached to those Expansion Hub motor channels.
- [ ] Confirm the encoder connectors and strain relief cannot contact moving gears.

### A3. Pod analog wiring

- [ ] Confirm one joiner cable is connected to the Control Hub physical analog connector labeled `0-1`.
- [ ] Confirm the left analog signal is channel 0 and named `absencleft`.
- [ ] Confirm the right analog signal is channel 1 and named `absencright`.
- [ ] Move only the left pod and confirm only `absencleft` changes.
- [ ] Move only the right pod and confirm only `absencright` changes.
- [ ] Confirm the two signals are not electrically tied together.
- [ ] Record connection information shown by the diagnostic tools:

```text
Analog joiner/cable notes:
Left analog connection:
Right analog connection:
Quadrature hub connection:
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

- [ ] Confirm the initial bevel pair for each motor is 1:1.
- [ ] Confirm four 16-tooth gears are installed as the driving gears.
- [ ] Confirm four 54-tooth gears are installed as the differential gears.
- [ ] Confirm each 54-tooth gear is rigidly attached to its 50-tooth gear.
- [ ] Confirm the two 50-tooth gears engage the common 19-tooth wheel gear.
- [ ] Confirm the right pod is rotated 180° mechanically but is not mirrored.
- [ ] Confirm the gears rotate freely without binding through the full allowed pod range.
- [ ] Confirm all gear set screws, hubs, shims, and structural fasteners are secure.

### B2. Geometry

- [ ] Measure and record wheel diameter:

```text
Measured wheel diameter: ______ mm
Code value currently used: 60 mm
```

- [ ] Measure and record pod wheel-center spacing:

```text
Measured pod spacing: ______ mm
Code value currently used: 362.96 mm
```

- [ ] Confirm both pods are centered at the documented left/right locations.
- [ ] Confirm the installed wheel/tread type and document any difference from the BOM.
- [ ] Confirm the wheel does not slip on its 8 mm REX/compatible mounting.

## C. Motor direction and differential behavior

These checks require the robot to be raised. Use low calibration speed.

### C1. Individual motor sanity

Use the appropriate motor diagnostic procedure and verify that each motor’s physical direction is understood before running the main drive.

| Motor | Expected physical response | Result/notes |
|---|---|---|
| `motor0` | Left pod lower-path motor direction known | |
| `motor1` | Left pod upper-path motor direction known | |
| `motor2` | Right pod lower-path motor direction known | |
| `motor3` | Right pod upper-path motor direction known | |

- [ ] Confirm all four programmed software directions remain `FORWARD`.
- [ ] Confirm equal positive commands propel the left pod in its directed robot-forward wheel-travel direction.
- [ ] Confirm equal positive commands propel the right pod in its directed robot-forward wheel-travel direction.
- [ ] Confirm left-positive/right-negative motor mixing steers each pod clockwise viewed from above.
- [ ] Confirm left-negative/right-positive steers each pod counterclockwise.
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
Left forward voltage: ______ V
```

- [ ] Record the stable raw analog angle shown by telemetry:

```text
Left forward raw degrees: ______ °
```

### D2. Right forward reference

- [ ] Select the right pod with RB.
- [ ] Align the right directed wheel-travel axis with robot-forward.
- [ ] Record the stable analog voltage:

```text
Right forward voltage: ______ V
```

- [ ] Record the stable raw analog angle shown by telemetry:

```text
Right forward raw degrees: ______ °
```

### D3. Analog direction and repeatability

For each pod, rotate slowly clockwise viewed from above and observe whether raw analog angle increases or decreases. Account for wraparound.

| Pod | Clockwise analog behavior | Programmed analog sign | Confirmed |
|---|---|---:|:---:|
| Left | increases / decreases | `+1` currently | [ ] |
| Right | increases / decreases | `+1` currently | [ ] |

- [ ] Repeat the forward alignment at least three times per pod.
- [ ] Record the observed repeatability:

```text
Left forward reference spread: ______ °
Right forward reference spread: ______ °
```

- [ ] Confirm the forward reference is not near an unsafe analog wrap boundary for the intended steering range.

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

- [ ] Confirm magnitude is approximately 1024 counts.
- [ ] Record the quadrature direction result:

```text
Left quadrature sign required: +1 / -1
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

- [ ] Confirm magnitude is approximately 1024 counts.
- [ ] Record the quadrature direction result:

```text
Right quadrature sign required: +1 / -1
```

### E3. Wheel-motion isolation

- [ ] Roll the left wheel without rotating the left pod and confirm the pod quadrature count does not change.
- [ ] Roll the right wheel without rotating the right pod and confirm the pod quadrature count does not change.
- [ ] If counts change, stop and inspect mechanical coupling or wiring before enabling drive.

## F. Motor encoder count and differential verification

Run `Motor Encoder Count Test`. This test applies no motor power.

Known values:

```text
Gearbox-output encoder: 145.1 pulses/revolution
Encoder-shaft encoder: 28 pulses/revolution
Planetary ratio:        5.2:1
External wheel ratio:   (16/54) × (50/19) = 0.779727
```

- [ ] Press A to set a count baseline.
- [ ] Manually rotate the left pod and record `motor0`/`motor1` count changes.
- [ ] Confirm the left motor-pair differential changes when pod azimuth changes.
- [ ] Manually rotate the right pod and record `motor2`/`motor3` count changes.
- [ ] Confirm the right motor-pair differential changes when pod azimuth changes.
- [ ] Confirm equal motor-path motion corresponds to wheel-drive motion.
- [ ] Confirm opposite motor-path motion corresponds to pod-steering motion.
- [ ] Record any sign or assembly mismatch:

```text
Left motor count observations:
Right motor count observations:
Differential gear mismatch or binding:
```

## G. Enter calibration values into source

After review, update `SwervePodEncoder.java`:

```java
public static final int LEFT_QUADRATURE_SIGN = ______;
public static final int RIGHT_QUADRATURE_SIGN = ______;
public static final int LEFT_ANALOG_SIGN = ______;
public static final int RIGHT_ANALOG_SIGN = ______;
public static final double LEFT_FORWARD_DEGREES = ______;
public static final double RIGHT_FORWARD_DEGREES = ______;
public static final boolean CALIBRATION_VERIFIED = true;
```

Before setting the final flag:

- [ ] Both forward values are finite and in the range 0° through less than 360°.
- [ ] Both analog signs are physically confirmed.
- [ ] Both quadrature signs are physically confirmed.
- [ ] Both full-revolution magnitudes are consistent with 1024 CPR.
- [ ] Values were independently reviewed by another person.
- [ ] `README.md`, `hardware.md`, and this task file contain the same accepted values.

## H. Main drive startup alignment

With calibration entered:

- [ ] Build and deploy the current debug APK.
- [ ] Confirm the main drive reports calibration ready.
- [ ] Place both pods away from forward so movement is observable.
- [ ] Start `Differential Swerve TeleOp` with the robot raised.
- [ ] Confirm both pods automatically steer toward their independent analog forward references.
- [ ] Confirm both pods stop within the configured alignment tolerance.
- [ ] Confirm the main drive refuses to proceed if a pod cannot align before timeout.
- [ ] Confirm startup seeds both quadrature trackers at zero angle.
- [ ] Restart the robot and repeat at least three times.
- [ ] Confirm the final startup pod-angle error is acceptable:

```text
Left startup error: ______ °
Right startup error: ______ °
```

## I. Low-speed powered drivetrain test

Run only after sections A–H are complete. Keep the robot raised for the first tests.

- [ ] Forward translation.
- [ ] Reverse translation.
- [ ] Robot-right strafe.
- [ ] Robot-left strafe.
- [ ] Clockwise chassis rotation.
- [ ] Counterclockwise chassis rotation.
- [ ] Translation plus clockwise rotation.
- [ ] Translation plus counterclockwise rotation.
- [ ] Release right stick and confirm zero requested rotation.
- [ ] Confirm pod targets are retained when requested pod velocity is zero.
- [ ] Confirm shortest-path steering reverses wheel direction correctly near 90°.
- [ ] Confirm analog wraparound does not cause an unexpected steering jump.
- [ ] Confirm motor velocities remain within the configured limit.
- [ ] Confirm stop behavior sets all four motor velocities to zero.
- [ ] Confirm a feedback fault stops the drive and requires restart.

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

- [ ] Hardware names and ports match the software.
- [ ] Differential gear assembly matches the documented topology.
- [ ] Motor directions and pod steering signs are confirmed.
- [ ] Analog forward references are recorded.
- [ ] Analog signs are recorded.
- [ ] Quadrature signs are recorded.
- [ ] Full-revolution quadrature counts are confirmed.
- [ ] Calibration constants are entered and reviewed.
- [ ] `CALIBRATION_VERIFIED` is true.
- [ ] Startup alignment succeeds repeatedly.
- [ ] Low-speed drive tests pass.
- [ ] Stop and fault behavior pass.
- [ ] Loaded speed, thermal, current, and tuning checks are documented.

## Final accepted values

Keep this section synchronized with `SwervePodEncoder.java` and `hardware.md` after commissioning.

```text
Left forward analog voltage: ______ V
Left forward raw degrees:    ______ °
Right forward analog voltage: _____ V
Right forward raw degrees:    _____ °
Left analog sign:             ______
Right analog sign:            ______
Left quadrature sign:         ______
Right quadrature sign:        ______
Left full-revolution counts:  ______
Right full-revolution counts: ______
Measured wheel diameter:      ______ mm
Measured pod spacing:         ______ mm
Final steering KP:             ______
Final steering KD:             ______
Final motor PIDF:              ______
Final safe drive limit:        ______
Final safe steering limit:     ______
Commissioned by:              ______
Commission date:               ______
```
