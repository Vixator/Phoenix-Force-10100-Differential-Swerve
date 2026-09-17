# Project Design

## Purpose

This project controls and commissions a two-pod differential-swerve FTC drivetrain. This document records the design decisions that explain why the code is structured and behaves as it does. Hardware specifications and wiring details are maintained in [hardware.md](hardware.md); the human-run calibration and commissioning procedure is maintained in [HUMAN_TASKS.md](HUMAN_TASKS.md). This document focuses on system architecture, software behavior, and coding requirements.

## Design Requirements

1. The robot shall use two differential-swerve pods, with two motors per pod.
2. The drive shall support robot-centric translation and chassis rotation.
3. Pod azimuth shall use the dedicated Melonbotics encoder quadrature outputs for runtime tracking.
4. Absolute analog encoder outputs shall provide the startup azimuth reference.
5. At startup, the main drive shall steer both pods to their independent analog forward references before accepting driver motion.
6. The drive shall remain disabled until encoder calibration has been physically measured and verified.
7. Diagnostics shall be read-only whenever they are intended for wiring or calibration work; motor-assisted alignment actions must be explicit, bounded, and abortable.
8. Hardware faults and invalid feedback shall fail safe by stopping drive commands and requiring an OpMode restart.
9. **Each independent robot subsystem or diagnostic function shall be implemented in its own OpMode file.** Do not combine unrelated subsystem tests into one large OpMode. Shared calculations and constants belong in reusable classes; hardware ownership, lifecycle, telemetry, and commands belong in the OpMode for that subsystem.

## Architecture

### OpModes

- `DifferentialSwerveTeleOp` owns the powered drivetrain, gamepad interpretation, hub reads, motor commands, pod tracking, and drive telemetry.
- `SwervePodEncoderTest` owns the combined analog/quadrature encoder commissioning workflow; its normal measurement path is read-only and its Y action is bounded selected-pod alignment.
- `PodAnalogEncoderTest` owns analog display and bounded selected-pod alignment to the forward reference.
- `MotorEncoderCountTest` owns the no-power motor encoder count measurement workflow.

New robot subsystems should follow the same separation. Examples include an intake test, lift test, vision test, or autonomous subsystem test: each should have a separate appropriately named OpMode rather than being added to the drivetrain OpMode.

### Shared class

`SwervePodEncoder` contains hardware-independent encoder math and calibration constants shared by the drive and encoder diagnostics. `PodAlignmentController` contains the shared bounded analog-feedback alignment state machine. Neither class acquires hardware, sends motor commands, or owns an OpMode lifecycle.

This separation keeps the math and alignment safety rules unit-testable and prevents the drive and diagnostics from developing different timeout, clamp, or wrap behavior.

## Hardware and Naming Decisions

### Drive motors

Four goBILDA `5203-2402-0005` motors are assigned to the Control Hub:

| Name | Port | Position |
|---|---:|---|
| `motor0` | 0 | Left pod, left motor |
| `motor1` | 1 | Left pod, right motor |
| `motor2` | 2 | Right pod, left motor |
| `motor3` | 3 | Right pod, right motor |

The motor specification used by the software is:

- 1150 RPM no-load output speed at 12 V
- 5.2:1 planetary gearbox
- 28 pulses per encoder-shaft revolution
- 145.1 pulses per gearbox-output revolution

The code uses the 145.1 gearbox-output value for velocity control because motor commands and telemetry are expressed at the gearbox output. Motor encoder positions are not used as the active pod-angle sensor.

### Pod encoders

Each Melonbotics Through Bore Encoder contains both sensing outputs and is mechanically 1:1 with pod azimuth:

- Quadrature: Expansion Hub motor channels 0 and 1, named `encoderleft` and `encoderright`
- Analog: Control Hub analog channels 0 and 1, named `absenc` and `absenc2`
- Quadrature resolution: 1024 CPR
- Analog range: 0–3.2 V

Raw controller counts are read directly so motor-channel direction settings cannot silently change encoder polarity.

## Mechanical Ratio

The programmed wheel-drive ratio is:

```text
(16 / 54) * (50 / 19) = 0.779727
```

The known differential setup is:

1. Each motor has an initial horizontal-to-vertical bevel pair with a 1:1 ratio.
2. Each motor then drives its own 16-tooth gear into a 54-tooth gear.
3. Each 54-tooth gear is rigidly attached to a 50-tooth gear.
4. The two 50-tooth gears engage the common 19-tooth wheel gear.
5. The left motor feeds the lower differential path and the right motor feeds the upper differential path.

Equal motor motion produces the wheel-drive component. The difference between the motor motions produces the pod-steering component. The ratio is used to estimate wheel speed and convert chassis rotation requests into normalized pod speeds. The estimate is theoretical; loaded speed, slip, and actual mechanical efficiency must be measured on the robot.

## Differential Motor Mixing

Each pod uses the same differential convention:

```text
left motor velocity  = wheel-drive component + steering component
right motor velocity = wheel-drive component - steering component
```

Equal motor velocities produce wheel drive. Opposite motor velocities produce pod steering. The right pod is physically rotated 180 degrees but is not treated as mirrored in software; its independent analog forward reference accounts for the mounting orientation.

The controller optimizes the requested pod vector to a maximum 90-degree steering error. If the desired vector is farther away, it reverses wheel direction and uses the equivalent shorter steering path. This reduces unnecessary pod rotation and is why individual motor signs cannot be assumed fixed for every driver command.

## Encoder Calibration and Tracking

At startup, the drive:

1. Reads both analog voltages.
2. Converts them to raw degrees with `volts / 3.2 * 360`.
3. Computes each signed shortest error to its independently measured forward reference.
4. Uses the known differential motor mixing to steer both pods toward those references with bounded velocity and a timeout.
5. Stops and refuses to drive if either pod cannot reach its reference or feedback becomes invalid.
6. Seeds both quadrature trackers at zero pod angle with fresh raw counts after alignment.

After startup, runtime angle tracking uses quadrature only:

```text
delta angle = delta counts * quadrature sign * 2π / 1024
```

Analog is not repeatedly used for correction because the analog output wraps at the voltage boundary and can introduce discontinuities during normal steering. Quadrature count differences are calculated as integers before conversion so signed rollover is handled correctly.

Forward references and signs are deliberately not guessed. Until `CALIBRATION_VERIFIED` is true and both forward references are finite, powered drive exits without commanding motion.

## Control Decisions

- **Robot-centric control:** avoids requiring an IMU or field heading reference.
- **Left stick translation:** direction selects the requested robot-relative pod vector; radial deadband prevents small joystick noise.
- **Right stick X rotation:** requests clockwise-positive chassis rotation about the pod midpoint; cubic shaping provides finer low-input control.
- **No heading hold:** releasing the turn stick requests zero chassis rotation rather than an automatic return or heading correction.
- **Steering priority:** steering commands retain authority when drive and steering compete for the motor-speed limit. Remaining headroom is allocated to wheel drive.
- **Cosine-squared alignment scaling:** wheel drive is reduced while a pod is misaligned with its requested vector.
- **Steering slew limiting:** limits abrupt changes in the steering command.
- **Measured-rate damping:** derivative steering feedback uses measured pod angular rate, avoiding target-step derivative spikes.

## Safety and Fault Handling

- Diagnostics do not apply motor power or reset hardware encoder counts.
- The drive commands zero velocity during initialization.
- The drive refuses to start without verified calibration.
- Invalid analog values, invalid hub bulk reads, or excessive loop delays stop the drive.
- A feedback fault requires an OpMode restart rather than automatic recovery.
- Shutdown attempts to stop every drive motor, even if one stop operation throws.
- Hardware names and ports are checked at runtime, including the requirement that pod quadrature inputs are on Expansion Hub channels 0 and 1 and are on a different hub from the drive motors.
- Software validity checks cannot guarantee that a disconnected analog sensor or frozen quadrature sensor is detected; physical commissioning remains required.

## Diagnostics Design

The diagnostics are intentionally split by purpose:

- Use `PodAnalogEncoderTest` to inspect analog voltage and absolute raw position; LB/RB select a pod, A performs bounded motor-assisted alignment to its forward reference, and B aborts.
- Use `MotorEncoderCountTest` to inspect motor encoder counts while rotating pods manually with no motor power. It reports gearbox-output and encoder-shaft conversions plus the motor-pair differential deltas for comparison with the known differential gear setup. Pod azimuth remains measured by the dedicated 1:1 Melonbotics encoder rather than inferred from motor counts.
- Use `SwervePodEncoderTest` to zero a selected quadrature display in software, measure a manual full revolution, compare analog against quadrature, or perform bounded alignment to the selected pod’s analog forward reference.

Calibration captures are session-only. They are reported back and then entered into the shared source constants after review rather than being silently persisted or automatically copied into the drive.

## Code Cleanliness Requirement

Every new feature must first be assigned to a subsystem. The implementation must then follow these rules:

- One subsystem or diagnostic purpose per OpMode file.
- No unrelated motor, sensor, or mechanism tests inside `DifferentialSwerveTeleOp`.
- Shared pure math belongs in a focused helper class with unit tests.
- Hardware lookup and actuator ownership remain local to the owning subsystem OpMode.
- OpModes must have clear names and FTC groups so the Driver Station menu remains understandable.
- Read-only diagnostics must not call actuator output, direction, mode, target-position, or reset APIs unless that operation is explicitly part of the diagnostic and documented. The alignment hotkeys are the documented exception and are bounded to a selected pod with an abort key and timeout.
- Changes that combine multiple subsystems into one file require a documented safety or lifecycle reason.

This requirement is part of the project design, not merely a style preference: separating subsystem ownership limits accidental hardware interaction, makes tests safer, and keeps commissioning tools usable without running the powered drivetrain.

## Verification Strategy

- Unit-test hardware-independent encoder math, wrapping, calibration validation, signs, and rollover.
- Build the TeamCode debug APK after adding or changing an OpMode.
- Complete and record the human-run steps in [HUMAN_TASKS.md](HUMAN_TASKS.md).
- Physically verify wiring, motor directions, analog polarity, quadrature polarity, counts per revolution, startup alignment, and loaded drivetrain behavior.
- Keep powered tests at reduced limits with the robot raised and clear of moving mechanisms.
- Update this document and `hardware.md` whenever a design assumption changes.
