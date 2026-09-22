# Project Design

## Purpose

This project controls and commissions a two-pod differential-swerve FTC drivetrain. This document records the design decisions that explain why the code is structured and behaves as it does. Hardware specifications and wiring details are maintained in [hardware.md](hardware.md); the human-run calibration and commissioning procedure is maintained in [HUMAN_TASKS.md](HUMAN_TASKS.md). This document focuses on system architecture, software behavior, and coding requirements.

## Design Requirements

1. The robot shall use two differential-swerve pods, with two motors per pod.
2. The drive shall support robot-centric translation and chassis rotation.
3. Pod azimuth shall use the dedicated Melonbotics encoder quadrature outputs for runtime tracking.
4. Absolute analog encoder outputs shall provide the startup azimuth reference.
5. During INIT, the main drive shall steer both pods to their independent analog forward references; driver motion starts only after both alignment and Start.
6. The drive shall remain disabled until encoder calibration has been physically measured and verified.
7. Diagnostics shall be read-only whenever they are intended for wiring or calibration work; motor-assisted alignment actions must be explicit, bounded, and abortable.
8. Invalid hub feedback shall stop motor commands before bounded retries. Persistent feedback faults shall latch stopped until OpMode restart; recovered runtime feedback requires neutral sticks and consistent pod angles before resuming.
9. **Each independent robot subsystem or diagnostic function shall be implemented in its own OpMode file.** Do not combine unrelated subsystem tests into one large OpMode. Shared calculations and constants belong in reusable classes. An OpMode may delegate hardware ownership and lifecycle to a subsystem-scoped owner, provided acquisition/cleanup remain within that OpMode lifetime and unrelated hardware is not initialized.

## Architecture

### OpModes

- `DifferentialSwerveTeleOp` owns gamepad interpretation and telemetry and scopes a `DifferentialSwerveRuntime` for hardware acquisition, alignment, feedback, and output cleanup.
- `SwervePodEncoderTest` owns the combined analog/quadrature encoder commissioning workflow; its normal measurement path is read-only and its Y action is bounded selected-pod alignment.
- `PodAnalogEncoderTest` owns analog display and bounded selected-pod alignment to the forward reference.
- `MotorEncoderDriveTest` owns the bounded individual-motor direction and module-response workflow.

New robot subsystems should follow the same separation. Examples include an intake test, lift test, vision test, or autonomous subsystem test: each should have a separate appropriately named OpMode rather than being added to the drivetrain OpMode.

### Shared class

`SwervePodEncoder` contains hardware-independent encoder math and calibration constants shared by the drive and encoder diagnostics. `PodAlignmentController` contains the shared bounded analog-feedback alignment state machine. Neither class acquires hardware, sends motor commands, or owns an OpMode lifecycle.

`DifferentialSwerveKinematics` converts robot-relative translation and clockwise-positive chassis rotation to pod vectors. `DifferentialSwervePodController` owns shortest-path optimization, PD steering, slew state, alignment scaling, and normalized motor mixing. Its clockwise-to-motor-steering conversion is shared with startup alignment and the individual pod diagnostic.

`SwerveDriverInput` owns radial translation deadband and cubic rotation shaping. The shared runtime handles initialization, alignment, analog-to-quadrature handoff, and cycle authorization through a small injectable `Host` boundary. `DifferentialSwerveHardware` implements that boundary with the SDK; `SwerveHubSession` coordinates snapshots and restores cache modes. Encoder state stays in the trackers. `SwerveTuning` provides validated live Dashboard settings; changed motor PIDF values are reapplied during runtime.

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
- Analog: Control Hub analog channels 0 and 1, named `absencleft` and `absencright`
- Encoder specification: 1024 CPR; software pod tracking scale: 4096 raw counts/revolution
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

The confirmed motor response is **counterclockwise** for positive steering in this mix. Encoder angles, errors, and measured rates are **clockwise-positive**, so `steering component = -(Kp * error - Kd * measuredRate)`. Both PD terms must be converted together. Applying this polarity only during analog alignment leaves the runtime loop in positive feedback. Chassis rotation remains clockwise-positive: right-stick right requests left-wheel forward/right-wheel backward, independently of the pod-steering polarity.

The controller reverses wheel direction to use the equivalent shorter steering path. Its first choice uses the 90° boundary; subsequent choices use a 3° hysteresis band (switch to reverse above 93°, back below 87°). This avoids chatter near perpendicular and permits up to 93° retained steering error. Wheel drive is suppressed whenever absolute error is at least 90°. Individual motor signs cannot be assumed fixed for every driver command.

## Encoder Calibration and Tracking

During INIT, the drive:

1. Reads both analog voltages.
2. Converts them to raw degrees with `volts / 3.2 * 360`.
3. Computes each signed shortest error to its independently measured forward reference.
4. Uses proportional velocity capped at 0.20 command, requiring 100 ms continuously within the 2° target window before the 5-second timeout, with correction polarity matched to the measured directions.
5. Stops both pods immediately if either fails. Completed pods are checked for drift while their partner finishes. The same 250 ms delay limit used in driving also applies during alignment.
6. Reports READY and waits at zero velocity for Start. The bounded alignment loop uses Stop as its cancellation condition, so it runs during INIT and finishes if Start is pressed early.
7. At Start, takes a fresh validated snapshot, verifies both pods are still within tolerance, and seeds quadrature with the measured residual analog angles and raw counts. The first runtime interval starts at that snapshot, not before the INIT wait. A pod moved out of tolerance requires reinitialization.

Analog validation accepts finite 0..3.3 V readings, allowing a margin above the nominal 3.2 V DAC endpoint. Conversion clamps that upper margin to 360° while retaining the 3.2 V scale and measured forward references. Invalid readings report the voltage; they are not silently replaced with zero.

After startup, runtime angle tracking uses quadrature only:

```text
delta angle = delta counts * quadrature sign * 2π / 4096
```

Analog is not repeatedly used for correction because the analog output wraps at the voltage boundary and can introduce discontinuities during normal steering. Quadrature count differences are calculated as integers before conversion so signed rollover is handled correctly.

The measured top-dead-center forward references are stored independently: left 0.122 V / 13.725° and right 0.258 V / 29.025°. Both analog voltages decrease during clockwise rotation, so both analog signs are `-1`. Both quadrature signs are confirmed as `+1` because clockwise rotation increased both raw counts. Wheel-drive direction and low-speed motion were previously confirmed; the refactored lifecycle requires a physical regression, and loaded behavior remains unverified. `CALIBRATION_VERIFIED` is true after physical encoder and combined-steering verification; powered drive now proceeds to bounded startup alignment. Powered commissioning and tuning remain separate human tasks.

## Control Decisions

- **Robot-centric TeleOp:** avoids requiring an IMU or field heading reference; Pedro autonomous uses Pinpoint.
- **Left stick translation:** direction selects the requested robot-relative pod vector; radial deadband prevents small joystick noise.
- **Right stick X rotation:** requests clockwise-positive chassis rotation about the pod midpoint; cubic shaping provides finer low-input control.
- **Full-speed pure rotation:** full stick maps to `2 * maxWheelSpeed / trackWidth`, so aligned pods request equal and opposite full wheel speeds using all four motors. Combined motion retains vector normalization and steering headroom. This calculated angular rate is a command scale, not a measured chassis speed.
- **No TeleOp heading hold:** releasing the turn stick requests zero chassis rotation rather than an automatic return or heading correction.
- **Steering priority:** steering commands retain authority when drive and steering compete for the motor-speed limit. Remaining headroom is allocated to wheel drive.
- **Cosine-squared alignment scaling:** wheel drive is reduced while a pod is misaligned with its requested vector.
- **Steering slew limiting:** limits abrupt changes in the steering command.
- **Measured-rate damping:** derivative steering feedback uses measured pod angular rate, avoiding target-step derivative spikes.
- **Bounded runtime steering:** preliminary defaults are Kp 0.5, Kd 0.01, steering limit 0.20, and slew rate 2.0/s. The runtime cap matches startup alignment; actual loaded tuning requires robot verification.

## Safety and Fault Handling

- Encoder measurement diagnostics do not reset hardware encoder counts; the individual motor diagnostic intentionally applies bounded power to one selected motor at a time.
- The drive starts at zero velocity, aligns the pods during INIT, then waits at zero velocity for Start.
- The drive refuses to start without verified calibration.
- Invalid analog values, invalid hub bulk reads, or excessive loop delays stop the drive.
- A failed bulk read commands zero velocity before retrying the entire hub set. `HubSnapshotReader` allows three attempts within 150 ms, with 10 ms pauses. No fake or mixed-attempt data reaches the controllers. The existing 250 ms loop limit still includes recovery time.
- Recovered runtime reads reset steering output state. TeleOp requires neutral sticks before resuming; autonomous latches stopped. Analog/quadrature disagreement above 10° latches a feedback fault, helping catch count resets after hub recovery; this check does not reseed or correct the tracker and cannot detect every reset.
- Persistent feedback faults stay in a stopped telemetry state until Stop/reinitialization. Hub identity and failure details are logged; cleanup attempts every motor stop and cache restoration even if another fails.
- Shutdown attempts to stop every drive motor, even if one stop operation throws.
- Hardware names and ports are checked at runtime, including the requirement that pod quadrature inputs are on Expansion Hub channels 0 and 1 and are on a different hub from the drive motors.
- Software validity checks cannot guarantee that a disconnected analog sensor or frozen quadrature sensor is detected; physical commissioning remains required.

## Diagnostics Design

The diagnostics are intentionally split by purpose:

- Use `PodAnalogEncoderTest` to inspect analog voltage and absolute raw position; LB/RB select a pod, A performs bounded motor-assisted alignment to its forward reference, and B aborts.
- Use `MotorEncoderDriveTest` to power one selected motor at a time at a bounded 25% maximum. D-pad selection and telemetry make individual motor direction and module response visible during physical commissioning.
- Use `SwervePodEncoderTest` to zero a selected quadrature display in software, measure a manual full revolution, compare analog against quadrature, or perform bounded alignment to the selected pod’s analog forward reference.

Calibration captures are session-only. They are reported back and then entered into the shared source constants after review rather than being silently persisted or automatically copied into the drive.

## Code Cleanliness Requirement

Every new feature must first be assigned to a subsystem. The implementation must then follow these rules:

- One subsystem or diagnostic purpose per OpMode file.
- No unrelated motor, sensor, or mechanism tests inside `DifferentialSwerveTeleOp`.
- Shared pure math belongs in a focused helper class with unit tests.
- Hardware lookup and actuator ownership remain scoped to the owning subsystem OpMode, including its shared runtime/hardware owner.
- OpModes must have clear names and FTC groups so the Driver Station menu remains understandable.
- Read-only diagnostics must not call actuator output, direction, mode, target-position, or reset APIs unless that operation is explicitly part of the diagnostic and documented. The alignment hotkeys are the documented exception and are bounded to a selected pod with an abort key and timeout.
- Changes that combine multiple subsystems into one file require a documented safety or lifecycle reason.

This requirement is part of the project design, not merely a style preference: separating subsystem ownership limits accidental hardware interaction, makes tests safer, and keeps commissioning tools usable without running the powered drivetrain.

## Verification Strategy

- Unit-test hardware-independent encoder math, wrapping, calibration validation, signs, and rollover.
- Regression-test startup/runtime polarity agreement, physical damping direction, chassis rotation, shortest-path reversal, target retention, motor limits, and convergence with a simple motor/encoder model.
- Build the TeamCode debug APK after adding or changing an OpMode.
- Complete and record the human-run steps in [HUMAN_TASKS.md](HUMAN_TASKS.md).
- Hardware configuration, separate quadrature hub, differential gear assembly, analog polarity, quadrature polarity, and individual motor responses are physically verified. Combined mixing and low-speed motion were previously reported working. Repeat startup/drive/fault checks after the runtime refactor; loaded dynamics remain unverified.
- Keep powered tests at reduced limits with the robot raised and clear of moving mechanisms.
- Update this document and `hardware.md` whenever a design assumption changes.

## Pedro 3.0.1 integration architecture

Autonomous code owns one `PedroAutoDrive` for the OpMode lifetime. That owner composes a
`SafePedroFollower`, `GuardedPinpointLocalizer`, `DifferentialSwerveDrivetrain`, and the
subsystem-scoped `DifferentialSwerveRuntime`. This is the documented exception to keeping hardware
lookup directly in an OpMode: the runtime owns only drivetrain devices and exists to make hub
snapshot, alignment, authorization, and cleanup behavior identical in TeleOp and autonomous.
OpModes still own construction, user interaction, Start/Stop, and `try/finally` lifetime.

The control sequence is one validated Lynx snapshot, one guarded Pinpoint update, one Pedro
calculation, then a two-pod prepare/validate/commit. Nonzero pod output is rejected without a fresh
runtime authorization. A runtime hub recovery pauses TeleOp until neutral but permanently cancels
autonomous. Stop, localization errors, stale samples, nonfinite commands, and partial output failures
all attempt zero velocity on every drive motor and retain BRAKE.

The shared coordinate contract is X forward, Y left, and positive yaw counterclockwise. Pinpoint
reports field-frame X/Y in inches. Pedro strafe is negated exactly once at the existing
forward/right kinematics boundary; Pedro normalized CCW turn is converted exactly once to the
existing CW rad/s convention. Pod encoder feedback remains clockwise-positive. No X-lock or active
FLOAT transition is part of normal operation.

`PinpointSettings.DIRECTIONS_VERIFIED`, its heading convention, and
`PedroFollowerConfig.MODEL_VERIFIED` are deliberate arming gates. Static pod, Pinpoint, manual-drive,
and characterization work is staged before enabling the disabled line, curve, heading, and template
OpModes. Field observations—not theoretical free speed—must populate the Foresight model.

## Review and verification record

[INTEGRATION_STATUS.md](INTEGRATION_STATUS.md) records the current software checks and remaining physical gates. Tests exercise the actual runtime state machine using an injected host, real pod/controller objects, fake motor boundaries, and a monotonic clock. This covers INIT/Start drift, output authorization, partial delivery, recovery, delayed reads, live caps, fault latching, and cleanup without relying on Android stub methods returning zero.

`DifferentialPod` validates finite combined motor targets, clears stale prepared output, checks phase/Stop/sample age before each write, and stops/latches after direct-call failures. Autonomous and static-angle tests retain the 0.35 combined cap even after live tuning changes; the TeleOp retains its full-scale profile. Cleanup attempts all actions and preserves secondary exceptions. Fault evidence survives close; lifecycle owners cannot be reused.

Pedro 3.0.1 Foresight finishes paths at the parametric endpoint. The configured endpoint timeout/tolerances alone do not enforce FOLLOW accuracy. `PedroPathResult` independently evaluates pose and velocity, and path diagnostics latch failed acceptance. A no-progress check observes path completion, endpoint distance, and heading improvement so a curve moving temporarily away from its endpoint is not mistaken for a stalled robot. Whole-path deadlines remain independent of library state.
