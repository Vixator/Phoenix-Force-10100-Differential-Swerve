package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

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
    public static volatile double HEADING_KP = 2.0; // rad/s per rad
    public static volatile double HEADING_KD = 0.15; // damping on Pinpoint angular velocity
    public static volatile double MAX_HOLD_RATE_RADIANS_PER_SECOND = 1.5;
    public static volatile double STEERING_SLEW_RATE = 8.0; // normalized steering output/s; <= 0 disables

    // Verify in INIT: signed heading must INCREASE on a clockwise chassis turn from above.
    // -1 converts the conventional upright Pinpoint's CCW-positive yaw to our CW-positive angles.
    // A fixed flat mounting yaw offset cancels when the Start reference is subtracted.
    public static double PINPOINT_HEADING_SIGN = -1.0; // sampled once at INIT, never changed during drive

    // ==================== VELOCITY PID CONSTANTS ====================
    public static volatile double VEL_PID_KP = 10.0;
    public static volatile double VEL_PID_KI = 0.0;
    public static volatile double VEL_PID_KD = 0.0;
    // REV velocity PIDF uses encoder ticks/s and a 32767 full-scale controller output, not volts/RPM.
    public static volatile double VEL_PID_KF = 32767.0 / MAX_MOTOR_TICKS_PER_SECOND;

    // ==================== STEERING CALIBRATION ====================
    // Carrier rotation = half the signed side-gear difference; first bevel stages are 1:1.
    // The 50:19 wheel gearing does NOT multiply carrier (pod) rotation. Verify with a measured turn.
    private static final double STEERING_RADIANS_PER_ENCODER_TICK =
            Math.PI * FIRST_STAGE_RATIO / MOTOR_TICKS_PER_REVOLUTION;

    // ==================== HARDWARE ====================
    // Control Hub motor ports:
    //   motor0 = Left Pod LEFT motor
    //   motor1 = Left Pod RIGHT motor
    //   motor2 = Right Pod LEFT motor
    //   motor3 = Right Pod RIGHT motor
    // Both pods: equal positive motors drive forward; left positive/right negative steers clockwise.
    private DcMotorEx leftMotorLeft;
    private DcMotorEx leftMotorRight;
    private DcMotorEx rightMotorLeft;
    private DcMotorEx rightMotorRight;
    private GoBildaPinpointDriver pinpoint;

    private int lastLeftMotorLeftPos;
    private int lastLeftMotorRightPos;
    private int lastRightMotorLeftPos;
    private int lastRightMotorRightPos;

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
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            stopAllMotors();
            applyVelocityPIDF();
            for (int i = 0; i < hubs.size(); i++) {
                hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            final double headingSign = PINPOINT_HEADING_SIGN;
            if (headingSign != -1.0 && headingSign != 1.0) {
                throw new IllegalArgumentException("PINPOINT_HEADING_SIGN must be -1 or +1");
            }
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            // Offsets/directions remain untouched until measured. X/Y position is NOT used by this teleop.
            pinpoint.resetPosAndIMU();
            long calibrationStarted = System.nanoTime();
            telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
            while (opModeInInit()) {
                pinpoint.update();
                double heading = pinpoint.getHeading(AngleUnit.DEGREES);
                telemetry.addLine("Keep stationary during calibration; wait at least 0.5 s for READY.");
                telemetry.addLine("Then verify CW chassis turn increases signed heading; return to start.");
                telemetry.addLine("Align chassis AND both pods forward before Start. INIT commands zero velocity.");
                telemetry.addLine("Odometry offsets/directions unverified; X/Y position not used.");
                telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
                telemetry.addData("Raw heading (deg)", heading);
                telemetry.addData("Signed CW heading (deg)", headingSign * heading);
                telemetry.addData("X encoder", pinpoint.getEncoderX());
                telemetry.addData("Y encoder", pinpoint.getEncoderY());
                telemetry.update();
                sleep(20); // INIT only, never in the drive loop
            }
            if (isStopRequested()) return;
            pinpoint.update();
            final double fieldHeadingOffset = headingSign * pinpoint.getHeading(AngleUnit.RADIANS);
            if (System.nanoTime() - calibrationStarted < 500_000_000L
                    || pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                    || !Double.isFinite(fieldHeadingOffset)) {
                telemetry.addLine("Start refused: Pinpoint not ready. Stop and initialize again.");
                telemetry.update();
                return;
            }

            // Capture encoder baselines only after the manually aligned modules are at Start.
            for (int i = 0; i < hubs.size(); i++) {
                if (hubs.get(i).getBulkData().isFake()) {
                    telemetry.addLine("Start refused: motor encoder bulk read failed.");
                    telemetry.update();
                    return;
                }
            }
            lastLeftMotorLeftPos = leftMotorLeft.getCurrentPosition();
            lastLeftMotorRightPos = leftMotorRight.getCurrentPosition();
            lastRightMotorLeftPos = rightMotorLeft.getCurrentPosition();
            lastRightMotorRightPos = rightMotorRight.getCurrentPosition();
            leftPodAngleRad = rightPodAngleRad = 0.0;
            double leftTargetAngle = 0.0;
            double rightTargetAngle = 0.0;
            double headingTarget = 0.0;
            boolean wasTurning = false;
            final double halfTrack = TRACK_WIDTH_METERS / 2.0;
            long lastLoop = System.nanoTime();
            long nextTelemetry = lastLoop;

            while (opModeIsActive()) {
                boolean encodersReady = true;
                // getBulkData refreshes and caches one snapshot, also used by the motor getters below.
                for (int i = 0; i < hubs.size(); i++) {
                    if (hubs.get(i).getBulkData().isFake()) encodersReady = false;
                }
                // Full update refreshes health AND yaw/rate. Heading-only update leaves health stale.
                pinpoint.update();
                double heading = wrapAngle(headingSign * pinpoint.getHeading(AngleUnit.RADIANS) - fieldHeadingOffset);
                double headingRate = headingSign * pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
                long now = System.nanoTime();
                loopSeconds = Math.max(0.000001, (now - lastLoop) * 1e-9);
                lastLoop = now;
                if (!encodersReady || pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                        || !Double.isFinite(heading) || !Double.isFinite(headingRate)
                        || loopSeconds > MAX_LOOP_SECONDS) {
                    stopAllMotors();
                    telemetry.addData("Drive stopped; restart required", pinpoint.getDeviceStatus());
                    telemetry.addData("Motor encoder read valid", encodersReady);
                    telemetry.addData("Loop seconds", loopSeconds);
                    telemetry.update();
                    break; // Detected faults require a restart; there is no robot-centric fallback.
                }
                updatePodAnglesFromMotorEncoders();

                double stickX = gamepad1.left_stick_x;
                double stickY = -gamepad1.left_stick_y;
                double stickMagnitude = Math.hypot(stickX, stickY);
                double deadband = clamp(DRIVE_DEADBAND, 0.0, 0.95);
                double driveSpeed = stickMagnitude > deadband
                        ? (Math.min(1.0, stickMagnitude) - deadband) / (1.0 - deadband) : 0.0;
                double inputScale = stickMagnitude > 0.0 ? driveSpeed / stickMagnitude : 0.0;
                double forward = stickY * inputScale;
                double strafe = stickX * inputScale;

                // Field -> robot, using CW-positive heading and right-positive strafe.
                // At +90 deg chassis heading, field-forward becomes robot-left (negative strafe).
                double cosHeading = Math.cos(heading);
                double sinHeading = Math.sin(heading);
                double robotForward = forward * cosHeading + strafe * sinHeading;
                double robotStrafe = -forward * sinHeading + strafe * cosHeading;

                double turn = gamepad1.right_stick_x;
                double turnDeadband = clamp(TURN_DEADBAND, 0.0, 0.95);
                if (Math.abs(turn) > turnDeadband) {
                    turn = Math.copySign((Math.abs(turn) - turnDeadband) / (1.0 - turnDeadband), turn);
                    turn = turn * turn * turn * clamp(TURN_INPUT_SCALE, 0.0, 1.0)
                            * Math.max(0.0, MAX_TURN_RATE_RADIANS_PER_SECOND);
                    headingTarget = heading;
                    wasTurning = true;
                } else {
                    // Capture release heading, not the preceding loop's heading or the field zero.
                    if (wasTurning) headingTarget = heading;
                    wasTurning = false;
                    double limit = Math.max(0.0, MAX_HOLD_RATE_RADIANS_PER_SECOND);
                    turn = clamp(HEADING_KP * wrapAngle(headingTarget - heading)
                            - HEADING_KD * headingRate, -limit, limit);
                }

                // Convert angular velocity (rad/s) to normalized wheel speed before mixing.
                double turnSpeed = turn * halfTrack / MAX_WHEEL_SPEED_METERS_PER_SECOND;
                double leftForward = robotForward + turnSpeed;
                double leftStrafe = robotStrafe;
                double rightForward = robotForward - turnSpeed;
                double rightStrafe = robotStrafe;
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
                    telemetry.addData("Field heading CW (deg)", Math.toDegrees(heading));
                    telemetry.addData("Hold heading CW (deg)", Math.toDegrees(headingTarget));
                    telemetry.addData("Left Pod Angle (deg)", Math.toDegrees(leftPodAngleRad));
                    telemetry.addData("Right Pod Angle (deg)", Math.toDegrees(rightPodAngleRad));
                    telemetry.addData("Left Target (deg)", Math.toDegrees(leftTargetAngle));
                    telemetry.addData("Right Target (deg)", Math.toDegrees(rightTargetAngle));
                    telemetry.addData("Drive Speed", driveSpeed);
                    telemetry.addData("Turn (rad/s CW)", turn);
                    telemetry.addData("Loop (ms)", loopSeconds * 1000.0);
                    telemetry.addData("L Vel L (ticks/s)", leftMotorLeft.getVelocity());
                    telemetry.addData("L Vel R (ticks/s)", leftMotorRight.getVelocity());
                    telemetry.addData("R Vel L (ticks/s)", rightMotorLeft.getVelocity());
                    telemetry.addData("R Vel R (ticks/s)", rightMotorRight.getVelocity());
                    telemetry.addData("L Enc L", lastLeftMotorLeftPos);
                    telemetry.addData("L Enc R", lastLeftMotorRightPos);
                    telemetry.addData("R Enc L", lastRightMotorLeftPos);
                    telemetry.addData("R Enc R", lastRightMotorRightPos);
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

    // Pod angle = integral of (motorLeft - motorRight) * steeringRatio
    private void updatePodAnglesFromMotorEncoders() {
        int currentLL = leftMotorLeft.getCurrentPosition();
        int currentLR = leftMotorRight.getCurrentPosition();
        int currentRL = rightMotorLeft.getCurrentPosition();
        int currentRR = rightMotorRight.getCurrentPosition();

        int deltaLL = currentLL - lastLeftMotorLeftPos;
        int deltaLR = currentLR - lastLeftMotorRightPos;
        int deltaRL = currentRL - lastRightMotorLeftPos;
        int deltaRR = currentRR - lastRightMotorRightPos;

        lastLeftMotorLeftPos = currentLL;
        lastLeftMotorRightPos = currentLR;
        lastRightMotorLeftPos = currentRL;
        lastRightMotorRightPos = currentRR;

        double leftDelta = ((double) deltaLL - deltaLR) * STEERING_RADIANS_PER_ENCODER_TICK;
        double rightDelta = ((double) deltaRL - deltaRR) * STEERING_RADIANS_PER_ENCODER_TICK;
        leftPodAngleRad += leftDelta;
        rightPodAngleRad += rightDelta;
        leftPodRateRad = leftDelta / loopSeconds;
        rightPodRateRad = rightDelta / loopSeconds;

        leftPodAngleRad = wrapAngle(leftPodAngleRad);
        rightPodAngleRad = wrapAngle(rightPodAngleRad);
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

        // Reduce wheel motion while misaligned; this is module alignment, NOT chassis heading hold.
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
        leftMotorLeft.setVelocity(0);
        leftMotorRight.setVelocity(0);
        rightMotorLeft.setVelocity(0);
        rightMotorRight.setVelocity(0);
    }
}
