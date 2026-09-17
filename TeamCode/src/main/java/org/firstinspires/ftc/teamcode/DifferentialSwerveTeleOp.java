package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp(name = "Differential Swerve TeleOp", group = "Differential Swerve")
public class DifferentialSwerveTeleOp extends LinearOpMode {

    // ==================== DRIVE TRAIN CONSTANTS ====================
    private static final double MOTOR_FREE_SPEED_RPM = 1150.0;
    private static final double MOTOR_TICKS_PER_REVOLUTION = 145.1;
    private static final double MAX_MOTOR_TICKS_PER_SECOND =
            MOTOR_FREE_SPEED_RPM * MOTOR_TICKS_PER_REVOLUTION / 60.0;
    private static final double FIRST_STAGE_RATIO = 16.0 / 54.0;
    private static final double SECOND_STAGE_RATIO = 50.0 / 19.0;
    // Differential pod: each motor uses a 1:1 bevel pair, then 16:54; each 54
    // gear is fixed to a 50 gear, and both 50 gears engage the common 19 wheel gear.
    // Equal motor components drive the wheel; their difference steers the pod.
    private static final double TOTAL_DRIVE_RATIO = FIRST_STAGE_RATIO * SECOND_STAGE_RATIO;
    private static final double WHEEL_DIAMETER_METERS = 0.06;
    private static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
    private static final double TRACK_WIDTH_METERS = 0.36296;
    private static final double MAX_WHEEL_SPEED_METERS_PER_SECOND =
            MOTOR_FREE_SPEED_RPM / 60.0 * TOTAL_DRIVE_RATIO * WHEEL_CIRCUMFERENCE_METERS;
    private static final int TELEMETRY_INTERVAL_MS = 100;
    private static final double MAX_LOOP_SECONDS = 0.25;

    // ==================== CONTROL CONSTANTS ====================
    // Preliminary gains, NOT robot-tuned. Dashboard dependency/@Config are not installed yet.
    public static volatile double STEERING_KP = 2.0;
    public static volatile double STEERING_KD = 0.003; // normalized output / (rad/s), damping on measurement
    public static volatile double MAX_DRIVE_POWER = 1.0;
    public static volatile double MAX_STEER_POWER = 1.0;
    public static volatile double TURN_INPUT_SCALE = 0.7;
    public static volatile double DRIVE_DEADBAND = 0.05;
    public static volatile double TURN_DEADBAND = 0.05;
    public static volatile double MAX_TURN_RATE_RADIANS_PER_SECOND = 3.0;
    public static volatile double STEERING_SLEW_RATE = 8.0; // normalized steering output/s; <= 0 disables

    // ==================== VELOCITY PID CONSTANTS ====================
    public static volatile double VEL_PID_KP = 10.0;
    public static volatile double VEL_PID_KI = 0.0;
    public static volatile double VEL_PID_KD = 0.0;
    // REV velocity PIDF uses encoder ticks/s and a 32767 full-scale controller output, not volts/RPM.
    public static volatile double VEL_PID_KF = 32767.0 / MAX_MOTOR_TICKS_PER_SECOND;

    // ==================== HARDWARE ====================
    // Control Hub motor ports:
    //   motor0 = Left Pod LEFT motor
    //   motor1 = Left Pod RIGHT motor
    //   motor2 = Right Pod LEFT motor
    //   motor3 = Right Pod RIGHT motor
    // Both pods: equal positive motors drive forward; left positive/right negative
    // steers clockwise through the known differential gear arrangement.
    private DcMotorEx leftMotorLeft;
    private DcMotorEx leftMotorRight;
    private DcMotorEx rightMotorLeft;
    private DcMotorEx rightMotorRight;
    // Expansion Hub encoder ports 0/1; these motor-channel handles are read-only.
    private DcMotor encoderleft;
    private DcMotor encoderright;
    private final SwervePodEncoder leftEncoder = new SwervePodEncoder(SwervePodEncoder.LEFT_QUADRATURE_SIGN);
    private final SwervePodEncoder rightEncoder = new SwervePodEncoder(SwervePodEncoder.RIGHT_QUADRATURE_SIGN);

    private double leftPodAngleRad;
    private double rightPodAngleRad;

    private double leftPodRateRad;
    private double rightPodRateRad;
    private double leftSteerPower;
    private double rightSteerPower;
    private double loopSeconds;
    private double lastVelP = Double.NaN;
    private double lastVelI = Double.NaN;
    private double lastVelD = Double.NaN;
    private double lastVelF = Double.NaN;

    @Override
    public void runOpMode() {
        // --- Motor initialization ---
        leftMotorLeft = hardwareMap.get(DcMotorEx.class, "motor0");
        leftMotorRight = hardwareMap.get(DcMotorEx.class, "motor1");
        rightMotorLeft = hardwareMap.get(DcMotorEx.class, "motor2");
        rightMotorRight = hardwareMap.get(DcMotorEx.class, "motor3");

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        LynxModule.BulkCachingMode[] previousModes = new LynxModule.BulkCachingMode[hubs.size()];
        for (int i = 0; i < hubs.size(); i++) {
            previousModes[i] = hubs.get(i).getBulkCachingMode();
        }
        try {
            // One-time setup only; no motor arrays are allocated in the drive loop.
            DcMotorEx[] motors = {leftMotorLeft, leftMotorRight, rightMotorLeft, rightMotorRight};
            for (DcMotorEx motor : motors) {
                // Electrical forward propels both aligned pods forward.
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // Velocity feedback does not require resetting absolute motor encoder counts.
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            stopAllMotors();
            applyVelocityPIDF();
            for (int i = 0; i < hubs.size(); i++) {
                hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
            encoderleft = hardwareMap.get(DcMotor.class, "encoderleft");
            encoderright = hardwareMap.get(DcMotor.class, "encoderright");
            if (encoderleft.getPortNumber() != 0 || encoderright.getPortNumber() != 1
                    || encoderleft.getController() != encoderright.getController()) {
                throw new IllegalArgumentException(
                        "Configure encoderleft/encoderright on Expansion Hub motor channels 0/1");
            }
            for (DcMotorEx motor : motors) {
                if (motor.getController() == encoderleft.getController()) {
                    throw new IllegalArgumentException("Pod encoders must be on a separate hub from drive motors");
                }
            }
            telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
            if (!SwervePodEncoder.calibrationReady()) {
                telemetry.addLine("Drive disabled: run Swerve Pod Encoder Test and set SwervePodEncoder calibration.");
                telemetry.addLine("Forward references, signs and raw counts/revolution must be verified first.");
                telemetry.update();
                waitForStart();
                return;
            }
            AnalogInput leftAbsolute = hardwareMap.get(AnalogInput.class, SwervePodEncoder.LEFT_ANALOG_NAME);
            AnalogInput rightAbsolute = hardwareMap.get(AnalogInput.class, SwervePodEncoder.RIGHT_ANALOG_NAME);
            telemetry.addLine("ROBOT-CENTRIC: left stick = translation, right stick X = turn.");
            telemetry.addLine("Start aligns both pods to their analog forward references, then tracks quadrature only.");
            telemetry.addLine("INIT commands zero velocity. Keep clear during automatic startup alignment.");
            telemetry.addData("Left pod encoder", encoderleft.getConnectionInfo());
            telemetry.addData("Right pod encoder", encoderright.getConnectionInfo());
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            // Before accepting driver input, actively align both pods to their measured
            // analog forward references. This establishes mechanical/electrical zero.
            if (!alignPodsToForward(leftAbsolute, rightAbsolute, hubs)) {
                telemetry.addLine("Start refused: pod forward alignment failed or was stopped.");
                telemetry.update();
                return;
            }
            int leftStartCount = encoderleft.getController().getMotorCurrentPosition(encoderleft.getPortNumber());
            int rightStartCount = encoderright.getController().getMotorCurrentPosition(encoderright.getPortNumber());
            leftPodAngleRad = 0.0;
            rightPodAngleRad = 0.0;
            leftEncoder.seed(0.0, leftStartCount);
            rightEncoder.seed(0.0, rightStartCount);
            double leftTargetAngle = 0.0;
            double rightTargetAngle = 0.0;
            final double halfTrack = TRACK_WIDTH_METERS / 2.0;
            long lastLoop = System.nanoTime();
            long nextTelemetry = lastLoop;

            while (opModeIsActive()) {
                boolean encodersReady = true;
                // getBulkData refreshes and caches one snapshot, also used by the motor getters below.
                for (int i = 0; i < hubs.size(); i++) {
                    if (hubs.get(i).getBulkData().isFake()) encodersReady = false;
                }
                long now = System.nanoTime();
                loopSeconds = Math.max(0.000001, (now - lastLoop) * 1e-9);
                lastLoop = now;
                if (!encodersReady || loopSeconds > MAX_LOOP_SECONDS) {
                    stopAllMotors();
                    telemetry.addLine("Drive stopped; restart required.");
                    telemetry.addData("Hub encoder read valid", encodersReady);
                    telemetry.addData("Loop seconds", loopSeconds);
                    telemetry.update();
                    break; // Do not resume automatically after a feedback fault.
                }
                updatePodAnglesFromEncoders();

                double stickX = gamepad1.left_stick_x;
                double stickY = -gamepad1.left_stick_y;
                double stickMagnitude = Math.hypot(stickX, stickY);
                double deadband = clamp(DRIVE_DEADBAND, 0.0, 0.95);
                double driveSpeed = stickMagnitude > deadband
                        ? (Math.min(1.0, stickMagnitude) - deadband) / (1.0 - deadband) : 0.0;
                double inputScale = stickMagnitude > 0.0 ? driveSpeed / stickMagnitude : 0.0;
                double forward = stickY * inputScale;
                double strafe = stickX * inputScale;

                double turn = gamepad1.right_stick_x;
                double turnDeadband = clamp(TURN_DEADBAND, 0.0, 0.95);
                if (Math.abs(turn) > turnDeadband) {
                    turn = Math.copySign((Math.abs(turn) - turnDeadband) / (1.0 - turnDeadband), turn);
                    turn = turn * turn * turn * clamp(TURN_INPUT_SCALE, 0.0, 1.0)
                            * Math.max(0.0, MAX_TURN_RATE_RADIANS_PER_SECOND);
                } else {
                    turn = 0.0;
                }

                // Robot-relative forward/right translation and clockwise-positive rotation.
                // Convert angular velocity (rad/s) to normalized wheel speed before mixing.
                double turnSpeed = turn * halfTrack / MAX_WHEEL_SPEED_METERS_PER_SECOND;
                double leftForward = forward + turnSpeed;
                double leftStrafe = strafe;
                double rightForward = forward - turnSpeed;
                double rightStrafe = strafe;
                double leftSpeed = Math.hypot(leftForward, leftStrafe);
                double rightSpeed = Math.hypot(rightForward, rightStrafe);
                double maxMagnitude = Math.max(1.0, Math.max(leftSpeed, rightSpeed));
                leftSpeed /= maxMagnitude;
                rightSpeed /= maxMagnitude;
                // A zero velocity vector has no azimuth: retain the last target rather than snap forward.
                if (leftSpeed > 1e-6) leftTargetAngle = Math.atan2(leftStrafe, leftForward);
                if (rightSpeed > 1e-6) rightTargetAngle = Math.atan2(rightStrafe, rightForward);
                if (isStopRequested()) break;
                if ((System.nanoTime() - now) * 1e-9 > MAX_LOOP_SECONDS) {
                    stopAllMotors();
                    telemetry.addLine("Drive stopped: control calculation timed out; restart required.");
                    telemetry.update();
                    break;
                }
                setPodStateVelocity(leftMotorLeft, leftMotorRight, leftPodAngleRad, leftTargetAngle, leftSpeed, true);
                setPodStateVelocity(rightMotorLeft, rightMotorRight, rightPodAngleRad, rightTargetAngle, rightSpeed, false);

                if (now >= nextTelemetry) {
                    nextTelemetry = now + TELEMETRY_INTERVAL_MS * 1_000_000L;
                    applyVelocityPIDF(); // Only writes to the hub when tuning values actually change.
                    telemetry.addLine("ROBOT-CENTRIC");
                    telemetry.addData("Left Pod Angle (deg)", Math.toDegrees(leftPodAngleRad));
                    telemetry.addData("Right Pod Angle (deg)", Math.toDegrees(rightPodAngleRad));
                    telemetry.addData("Left requested vector (deg)", Math.toDegrees(leftTargetAngle));
                    telemetry.addData("Right requested vector (deg)", Math.toDegrees(rightTargetAngle));
                    telemetry.addData("Drive Speed", driveSpeed);
                    telemetry.addData("Turn (rad/s CW)", turn);
                    telemetry.addData("Loop (ms)", loopSeconds * 1000.0);
                    telemetry.addData("L Vel L (ticks/s)", leftMotorLeft.getVelocity());
                    telemetry.addData("L Vel R (ticks/s)", leftMotorRight.getVelocity());
                    telemetry.addData("R Vel L (ticks/s)", rightMotorLeft.getVelocity());
                    telemetry.addData("R Vel R (ticks/s)", rightMotorRight.getVelocity());
                    telemetry.addData("Left pod encoder (counts)", leftEncoder.getCount());
                    telemetry.addData("Right pod encoder (counts)", rightEncoder.getCount());
                    telemetry.update();
                }
            }
        } finally {
            try {
                stopAllMotors();
            } finally {
                for (int i = 0; i < hubs.size(); i++) {
                    hubs.get(i).setBulkCachingMode(previousModes[i]);
                }
            }
        }
    }

    private boolean alignPodsToForward(AnalogInput leftAbsolute, AnalogInput rightAbsolute,
                                        List<LynxModule> hubs) {
        PodAlignmentController leftAlignment = new PodAlignmentController(
                SwervePodEncoder.LEFT_FORWARD_DEGREES, SwervePodEncoder.LEFT_ANALOG_SIGN);
        PodAlignmentController rightAlignment = new PodAlignmentController(
                SwervePodEncoder.RIGHT_FORWARD_DEGREES, SwervePodEncoder.RIGHT_ANALOG_SIGN);
        leftAlignment.start();
        rightAlignment.start();
        long lastSample = System.nanoTime();
        while (opModeIsActive() && !isStopRequested()) {
            boolean validSnapshot = true;
            for (LynxModule hub : hubs) {
                if (hub.getBulkData().isFake()) validSnapshot = false;
            }
            double leftVolts = leftAbsolute.getVoltage();
            double rightVolts = rightAbsolute.getVoltage();
            double seconds = Math.max(0.0, (System.nanoTime() - lastSample) * 1e-9);
            lastSample = System.nanoTime();
            if (!validSnapshot) {
                leftAlignment.abort("invalid hub snapshot");
                rightAlignment.abort("invalid hub snapshot");
            } else {
                leftAlignment.step(leftVolts, seconds);
                rightAlignment.step(rightVolts, seconds);
            }
            telemetry.addLine("ALIGNING PODS TO ANALOG FORWARD REFERENCES");
            telemetry.addData("Left target/error (deg)", "%.2f / %.2f",
                    leftAlignment.getTargetDegrees(), leftAlignment.getErrorDegrees());
            telemetry.addData("Right target/error (deg)", "%.2f / %.2f",
                    rightAlignment.getTargetDegrees(), rightAlignment.getErrorDegrees());
            telemetry.addData("Alignment status", leftAlignment.getStatus() + " / " + rightAlignment.getStatus());
            telemetry.update();
            if (!leftAlignment.isActive() && !rightAlignment.isActive()) {
                stopAllMotors();
                return leftAlignment.isComplete() && rightAlignment.isComplete();
            }
            setAnalogSteeringVelocity(leftAlignment.getCommand(), rightAlignment.getCommand());
            idle();
        }
        stopAllMotors();
        return false;
    }

    private void setAnalogSteeringVelocity(double leftSteer, double rightSteer) {
        leftMotorLeft.setVelocity(leftSteer * MAX_MOTOR_TICKS_PER_SECOND);
        leftMotorRight.setVelocity(-leftSteer * MAX_MOTOR_TICKS_PER_SECOND);
        rightMotorLeft.setVelocity(rightSteer * MAX_MOTOR_TICKS_PER_SECOND);
        rightMotorRight.setVelocity(-rightSteer * MAX_MOTOR_TICKS_PER_SECOND);
    }

    private void updatePodAnglesFromEncoders() {
        int currentLeft = encoderleft.getController().getMotorCurrentPosition(encoderleft.getPortNumber());
        int currentRight = encoderright.getController().getMotorCurrentPosition(encoderright.getPortNumber());
        leftEncoder.update(currentLeft, loopSeconds);
        rightEncoder.update(currentRight, loopSeconds);
        leftPodAngleRad = leftEncoder.getAngleRadians();
        rightPodAngleRad = rightEncoder.getAngleRadians();
        leftPodRateRad = leftEncoder.getRateRadiansPerSecond();
        rightPodRateRad = rightEncoder.getRateRadiansPerSecond();
    }

    // Velocity-based pod control with PD steering
    private void setPodStateVelocity(DcMotorEx motorLeft, DcMotorEx motorRight,
                                     double currentAngleRad, double targetAngleRad,
                                     double targetSpeed, boolean isLeftPod) {
        double angleError = wrapAngle(targetAngleRad - currentAngleRad);

        // Equivalent wheel vector with at most 90 degrees of module rotation.
        if (Math.abs(angleError) > Math.PI / 2.0) {
            angleError -= Math.copySign(Math.PI, angleError);
            targetSpeed = -targetSpeed;
        }

        // D on measured pod rate avoids target-step/optimization-branch derivative kicks.
        double angleErrorDeriv = -(isLeftPod ? leftPodRateRad : rightPodRateRad);
        double steerLimit = clamp(MAX_STEER_POWER, 0.0, 1.0);
        double steerPower = clamp(angleError * STEERING_KP + angleErrorDeriv * STEERING_KD,
                -steerLimit, steerLimit);
        double slewRate = STEERING_SLEW_RATE;
        if (slewRate > 0.0) {
            double previous = isLeftPod ? leftSteerPower : rightSteerPower;
            double step = slewRate * loopSeconds;
            steerPower = clamp(steerPower, previous - step, previous + step);
        }
        steerPower = clamp(steerPower, -steerLimit, steerLimit);
        if (isLeftPod) {
            leftSteerPower = steerPower;
        } else {
            rightSteerPower = steerPower;
        }

        // Reduce wheel motion while the pod is misaligned with its requested travel direction.
        double headingScale = Math.max(0.0, Math.cos(angleError));
        headingScale = headingScale * headingScale;

        double driveLimit = clamp(MAX_DRIVE_POWER, 0.0, 1.0);
        double drivePower = targetSpeed * headingScale * driveLimit;
        // Preserve steering authority. Independent motor clipping distorts drive/steer decomposition.
        double driveHeadroom = 1.0 - Math.abs(steerPower);
        drivePower = clamp(drivePower, -driveHeadroom, driveHeadroom);
        double targetVelocityTicks = drivePower * MAX_MOTOR_TICKS_PER_SECOND;
        double steerVelocityTicks = steerPower * MAX_MOTOR_TICKS_PER_SECOND;

        // Differential: left = drive + steer, right = drive - steer
        double leftVel = targetVelocityTicks + steerVelocityTicks;
        double rightVel = targetVelocityTicks - steerVelocityTicks;
        if (!Double.isFinite(leftVel) || !Double.isFinite(rightVel)) {
            throw new IllegalArgumentException("Nonfinite motor command; check tuning values");
        }

        // Set velocity targets (motor controller handles PID internally)
        motorLeft.setVelocity(leftVel);
        motorRight.setVelocity(rightVel);
    }

    private void applyVelocityPIDF() {
        double p = VEL_PID_KP, i = VEL_PID_KI, d = VEL_PID_KD, f = VEL_PID_KF;
        if (!Double.isFinite(p) || !Double.isFinite(i) || !Double.isFinite(d) || !Double.isFinite(f)
                || p < 0.0 || i < 0.0 || d < 0.0 || f < 0.0) {
            throw new IllegalArgumentException("Velocity PIDF must be finite and nonnegative");
        }
        if (p == lastVelP && i == lastVelI && d == lastVelD && f == lastVelF) return;
        leftMotorLeft.setVelocityPIDFCoefficients(p, i, d, f);
        leftMotorRight.setVelocityPIDFCoefficients(p, i, d, f);
        rightMotorLeft.setVelocityPIDFCoefficients(p, i, d, f);
        rightMotorRight.setVelocityPIDFCoefficients(p, i, d, f);
        lastVelP = p;
        lastVelI = i;
        lastVelD = d;
        lastVelF = f;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double wrapAngle(double angle) {
        angle %= 2.0 * Math.PI;
        if (angle <= -Math.PI) angle += 2.0 * Math.PI;
        if (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    private void stopAllMotors() {
        // A failure on one motor must not prevent stop attempts on the others.
        RuntimeException failure = null;
        DcMotorEx[] motors = {leftMotorLeft, leftMotorRight, rightMotorLeft, rightMotorRight};
        for (DcMotorEx motor : motors) {
            try {
                motor.setVelocity(0);
            } catch (RuntimeException exception) {
                if (failure == null) failure = exception;
            }
        }
        if (failure != null) throw failure;
    }
}
