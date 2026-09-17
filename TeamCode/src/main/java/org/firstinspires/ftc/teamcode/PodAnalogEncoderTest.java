package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Analog encoder display and bounded pod-forward alignment diagnostic.
 * LB selects the left pod, RB selects the right pod, A aligns the selected pod,
 * and B aborts alignment. The configured forward references are used when they
 * are finite; otherwise electrical analog zero is used for commissioning.
 */
@TeleOp(name = "Pod Analog Encoder Test", group = "Differential Swerve")
public class PodAnalogEncoderTest extends OpMode {
    private static final double MOTOR_TICKS_PER_SECOND = 2781.0833;

    private AnalogInput leftAbsolute;
    private AnalogInput rightAbsolute;
    private DcMotorEx leftMotorLeft;
    private DcMotorEx leftMotorRight;
    private DcMotorEx rightMotorLeft;
    private DcMotorEx rightMotorRight;

    private boolean selectedLeft = true;
    private boolean aligning;
    private long lastAlignmentNanos;
    private PodAlignmentController alignment;
    private boolean wasA;
    private boolean wasB;
    private boolean wasLeftBumper;
    private boolean wasRightBumper;

    @Override
    public void init() {
        leftAbsolute = hardwareMap.get(AnalogInput.class, SwervePodEncoder.LEFT_ANALOG_NAME);
        rightAbsolute = hardwareMap.get(AnalogInput.class, SwervePodEncoder.RIGHT_ANALOG_NAME);
        leftMotorLeft = getMotor("motor0");
        leftMotorRight = getMotor("motor1");
        rightMotorLeft = getMotor("motor2");
        rightMotorRight = getMotor("motor3");
        telemetry.setMsTransmissionInterval(100);
    }

    private DcMotorEx getMotor(String name) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    @Override
    public void loop() {
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        if (leftBumper && !wasLeftBumper) {
            selectedLeft = true;
            if (aligning) stopAlignment("pod selection changed");
        }
        if (rightBumper && !wasRightBumper) {
            selectedLeft = false;
            if (aligning) stopAlignment("pod selection changed");
        }
        if (b && !wasB) stopAlignment("operator abort");
        if (a && !wasA) startAlignment();
        wasLeftBumper = leftBumper;
        wasRightBumper = rightBumper;
        wasA = a;
        wasB = b;

        double leftVolts = leftAbsolute.getVoltage();
        double rightVolts = rightAbsolute.getVoltage();
        if (aligning) updateAlignment(leftVolts, rightVolts);
        else stopAllMotors();

        displayEncoder("Left", leftAbsolute, leftVolts, true);
        displayEncoder("Right", rightAbsolute, rightVolts, false);
        telemetry.addData("Selected pod", selectedLeft ? "LEFT (LB)" : "RIGHT (RB)");
        telemetry.addData("Alignment", aligning ? "ACTIVE - A target / B abort" : "idle");
        if (alignment != null) telemetry.addData("Alignment status", alignment.getStatus());
        telemetry.addLine("A: steer selected pod to its configured forward analog reference.");
        telemetry.addLine("LB/RB: select pod. B: stop. No drive/wheel command is issued.");
        telemetry.addLine("Targets are module-specific raw degrees; if unset, target is analog 0 degrees.");
        telemetry.update();
    }

    private void startAlignment() {
        alignment = new PodAlignmentController(
                SwervePodEncoder.forwardTargetDegrees(selectedLeft),
                selectedLeft ? SwervePodEncoder.LEFT_ANALOG_SIGN : SwervePodEncoder.RIGHT_ANALOG_SIGN);
        alignment.start();
        aligning = true;
        lastAlignmentNanos = System.nanoTime();
    }

    private void updateAlignment(double leftVolts, double rightVolts) {
        double volts = selectedLeft ? leftVolts : rightVolts;
        double seconds = Math.max(0.0, (System.nanoTime() - lastAlignmentNanos) * 1e-9);
        lastAlignmentNanos = System.nanoTime();
        alignment.step(volts, seconds);
        telemetry.addData("Alignment target/error (deg)", "%.2f / %.2f",
                alignment.getTargetDegrees(), alignment.getErrorDegrees());
        if (alignment.isActive()) {
            setSteeringVelocity(alignment.getCommand(), selectedLeft);
        } else {
            aligning = false;
            stopAllMotors();
        }
    }

    private void displayEncoder(String label, AnalogInput encoder, double volts, boolean leftPod) {
        telemetry.addData(label + " connection", encoder.getConnectionInfo());
        telemetry.addData(label + " voltage (V)", "%.4f", volts);
        if (SwervePodEncoder.validVoltage(volts)) {
            double raw = SwervePodEncoder.rawDegrees(volts);
            double target = SwervePodEncoder.forwardTargetDegrees(leftPod);
            double error = SwervePodEncoder.analogErrorDegrees(volts, target,
                    leftPod ? SwervePodEncoder.LEFT_ANALOG_SIGN : SwervePodEncoder.RIGHT_ANALOG_SIGN);
            telemetry.addData(label + " raw position (deg)", "%.3f", raw);
            telemetry.addData(label + " target/error (deg)", "%.3f / %.3f", target, error);
            telemetry.addData(label + " distance to wrap (deg)", "CW %.1f / CCW %.1f",
                    360.0 - raw, raw);
        } else {
            telemetry.addData(label + " raw position", "INVALID: expected 0..3.2 V");
        }
    }

    private void setSteeringVelocity(double steer, boolean leftPod) {
        double leftVelocity = steer * MOTOR_TICKS_PER_SECOND;
        double rightVelocity = -leftVelocity;
        if (leftPod) {
            leftMotorLeft.setVelocity(leftVelocity);
            leftMotorRight.setVelocity(rightVelocity);
        } else {
            rightMotorLeft.setVelocity(leftVelocity);
            rightMotorRight.setVelocity(rightVelocity);
        }
    }

    private void stopAlignment(String reason) {
        if (alignment != null && alignment.isActive()) alignment.abort(reason);
        aligning = false;
        stopAllMotors();
    }

    private void stopAllMotors() {
        if (leftMotorLeft == null) return;
        leftMotorLeft.setVelocity(0);
        leftMotorRight.setVelocity(0);
        rightMotorLeft.setVelocity(0);
        rightMotorRight.setVelocity(0);
    }

    @Override
    public void stop() {
        stopAllMotors();
    }
}
