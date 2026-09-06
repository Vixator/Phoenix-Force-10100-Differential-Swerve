package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Differential Swerve TeleOp", group = "Differential Swerve")
public class DifferentialSwerveTeleOp extends LinearOpMode {

    // ==================== DRIVE TRAIN CONSTANTS ====================
    private static final double MOTOR_FREE_SPEED_RPM = 1150.0;
    private static final double FIRST_STAGE_RATIO = 16.0 / 54.0;
    private static final double SECOND_STAGE_RATIO = 50.0 / 19.0;
    private static final double TOTAL_DRIVE_RATIO = FIRST_STAGE_RATIO * SECOND_STAGE_RATIO;
    private static final double WHEEL_DIAMETER_METERS = 0.06;
    private static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
    private static final double TRACK_WIDTH_METERS = 0.363354524;

    // ==================== CONTROL CONSTANTS ====================
    private static final double STEERING_KP = 2.0;
    private static final double MAX_DRIVE_POWER = 1.0;
    private static final double MAX_STEER_POWER = 1.0;
    private static final double TURN_INPUT_SCALE = 0.7;

    // ==================== STEERING CALIBRATION ====================
    // TODO: Calibrate this for your actual steering gear ratio.
    // Pod angle += (deltaMotorLeft - deltaMotorRight) * STEERING_RADIANS_PER_ENCODER_TICK
    private static final double STEERING_RADIANS_PER_ENCODER_TICK = (2.0 * Math.PI) / 5600.0;

    // ==================== HARDWARE ====================
    // Control Hub motor ports:
    //   motor0 = Left Pod LEFT motor
    //   motor1 = Left Pod RIGHT motor
    //   motor2 = Right Pod LEFT motor
    //   motor3 = Right Pod RIGHT motor
    // Right pod is flipped 180° relative to left pod.
    private DcMotorEx leftMotorLeft;
    private DcMotorEx leftMotorRight;
    private DcMotorEx rightMotorLeft;
    private DcMotorEx rightMotorRight;

    private int lastLeftMotorLeftPos;
    private int lastLeftMotorRightPos;
    private int lastRightMotorLeftPos;
    private int lastRightMotorRightPos;

    private double leftPodAngleRad;
    private double rightPodAngleRad;

    @Override
    public void runOpMode() {
        // --- Motor initialization ---
        leftMotorLeft  = hardwareMap.get(DcMotorEx.class, "motor0");
        leftMotorRight = hardwareMap.get(DcMotorEx.class, "motor1");
        rightMotorLeft = hardwareMap.get(DcMotorEx.class, "motor2");
        rightMotorRight = hardwareMap.get(DcMotorEx.class, "motor3");

        // Left pod: both motors FORWARD
        leftMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Right pod: flipped 180°, so motors spin opposite for same robot motion
        rightMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Initialize pod angle tracking from motor encoders ---
        lastLeftMotorLeftPos  = leftMotorLeft.getCurrentPosition();
        lastLeftMotorRightPos = leftMotorRight.getCurrentPosition();
        lastRightMotorLeftPos = rightMotorLeft.getCurrentPosition();
        lastRightMotorRightPos = rightMotorRight.getCurrentPosition();
        leftPodAngleRad = 0.0;
        rightPodAngleRad = 0.0;

        telemetry.addLine("Align pods straight forward, then press Start");
        telemetry.addLine("Controls: RT=forward, LT=reverse, L-stick=wheel angle, R-stick=turn");
        telemetry.update();

        waitForStart();

        // Re-zero at start
        lastLeftMotorLeftPos  = leftMotorLeft.getCurrentPosition();
        lastLeftMotorRightPos = leftMotorRight.getCurrentPosition();
        lastRightMotorLeftPos = rightMotorLeft.getCurrentPosition();
        lastRightMotorRightPos = rightMotorRight.getCurrentPosition();
        leftPodAngleRad = 0.0;
        rightPodAngleRad = 0.0;

        while (opModeIsActive()) {
            updatePodAnglesFromMotorEncoders();

            // --- Driver input ---
            // Triggers: RT = forward, LT = backward (reverse)
            double forward = gamepad1.right_trigger;
            double backward = gamepad1.left_trigger;
            double driveSpeed = forward - backward;  // +1.0 to -1.0

            // Left stick: direction wheels face (angle)
            double stickX = gamepad1.left_stick_x;
            double stickY = -gamepad1.left_stick_y;
            double wheelAngle = Math.atan2(stickX, stickY);  // 0 = forward, π/2 = right, etc.

            // Right stick X: robot rotation
            double turn = gamepad1.right_stick_x;
            turn = turn * turn * turn * TURN_INPUT_SCALE;  // cubic shaping

            // If stick near center, default to straight forward/backward based on driveSpeed sign
            if (Math.hypot(stickX, stickY) < 0.1) {
                wheelAngle = (driveSpeed >= 0) ? 0.0 : Math.PI;
            }

            // --- Differential swerve kinematics ---
            double halfTrack = TRACK_WIDTH_METERS / 2.0;

            // Each pod gets same wheel angle, but turn adds differential
            double leftForward = driveSpeed * Math.cos(wheelAngle) - turn * halfTrack;
            double leftStrafe = driveSpeed * Math.sin(wheelAngle);
            double rightForward = driveSpeed * Math.cos(wheelAngle) + turn * halfTrack;
            double rightStrafe = driveSpeed * Math.sin(wheelAngle);

            double leftSpeed = Math.hypot(leftForward, leftStrafe);
            double rightSpeed = Math.hypot(rightForward, rightStrafe);
            double maxMagnitude = Math.max(1.0, Math.max(leftSpeed, rightSpeed));

            leftSpeed /= maxMagnitude;
            rightSpeed /= maxMagnitude;

            double leftTargetAngle = Math.atan2(leftStrafe, leftForward);
            double rightTargetAngle = Math.atan2(rightStrafe, rightForward);

            // Right pod is flipped 180°: its "forward" is opposite, so add π to target angle
            rightTargetAngle = wrapAngle(rightTargetAngle + Math.PI);

            // --- Set pod states ---
            setPodState(leftMotorLeft, leftMotorRight, leftPodAngleRad, leftTargetAngle, leftSpeed);
            setPodState(rightMotorLeft, rightMotorRight, rightPodAngleRad, rightTargetAngle, rightSpeed);

            // --- Telemetry ---
            telemetry.addData("Left Pod Angle (deg)", Math.toDegrees(leftPodAngleRad));
            telemetry.addData("Right Pod Angle (deg)", Math.toDegrees(rightPodAngleRad));
            telemetry.addData("Drive Speed", driveSpeed);
            telemetry.addData("Wheel Angle (deg)", Math.toDegrees(wheelAngle));
            telemetry.addData("Turn", turn);
            telemetry.addData("Left Target (deg)", Math.toDegrees(leftTargetAngle));
            telemetry.addData("Right Target (deg)", Math.toDegrees(rightTargetAngle));
            telemetry.addData("Left Speed", leftSpeed);
            telemetry.addData("Right Speed", rightSpeed);
            telemetry.addData("L Enc L", leftMotorLeft.getCurrentPosition());
            telemetry.addData("L Enc R", leftMotorRight.getCurrentPosition());
            telemetry.addData("R Enc L", rightMotorLeft.getCurrentPosition());
            telemetry.addData("R Enc R", rightMotorRight.getCurrentPosition());
            telemetry.update();
        }

        stopAllMotors();
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

        lastLeftMotorLeftPos  = currentLL;
        lastLeftMotorRightPos = currentLR;
        lastRightMotorLeftPos = currentRL;
        lastRightMotorRightPos = currentRR;

        // Differential steering: pod rotation ∝ (motorLeft - motorRight)
        // Right pod motors are reversed in hardware, so flip the delta sign to match left pod convention
        leftPodAngleRad += (deltaLL - deltaLR) * STEERING_RADIANS_PER_ENCODER_TICK;
        rightPodAngleRad += (deltaRR - deltaRL) * STEERING_RADIANS_PER_ENCODER_TICK;

        leftPodAngleRad = wrapAngle(leftPodAngleRad);
        rightPodAngleRad = wrapAngle(rightPodAngleRad);
    }

    private void setPodState(DcMotorEx motorLeft, DcMotorEx motorRight, double currentAngleRad, double targetAngleRad, double targetSpeed) {
        double angleError = wrapAngle(targetAngleRad - currentAngleRad);

        // Optimize: reverse drive if error > 90°
        if (Math.abs(angleError) > Math.PI / 2.0) {
            targetAngleRad = wrapAngle(targetAngleRad + Math.PI);
            targetSpeed = -targetSpeed;
            angleError = wrapAngle(targetAngleRad - currentAngleRad);
        }

        double steerPower = clamp(angleError * STEERING_KP, -MAX_STEER_POWER, MAX_STEER_POWER);

        double headingScale = Math.max(0.0, Math.cos(angleError));
        double drivePower = clamp(targetSpeed * headingScale, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

        // Differential drive: motorLeft = drive + steer, motorRight = drive - steer
        double motorLeftPower = clamp(drivePower + steerPower, -1.0, 1.0);
        double motorRightPower = clamp(drivePower - steerPower, -1.0, 1.0);

        motorLeft.setPower(motorLeftPower);
        motorRight.setPower(motorRightPower);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double wrapAngle(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    private void stopAllMotors() {
        leftMotorLeft.setPower(0.0);
        leftMotorRight.setPower(0.0);
        rightMotorLeft.setPower(0.0);
        rightMotorRight.setPower(0.0);
    }
}