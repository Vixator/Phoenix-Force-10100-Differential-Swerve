package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

/**
 * Combined pod encoder commissioning diagnostic.
 *
 * LB/RB select a pod. A zeroes the selected quadrature display, X starts and
 * finishes a manual one-revolution count measurement, Y steers the selected pod
 * to its analog forward reference, and B aborts alignment. Quadrature zeroing
 * never resets the Expansion Hub hardware counter.
 */
@TeleOp(name = "Swerve Pod Encoder Test", group = "Differential Swerve")
public class SwervePodEncoderTest extends OpMode {
    private static final double MOTOR_TICKS_PER_SECOND = 2781.0833;

    private DcMotor encoderleft;
    private DcMotor encoderright;
    private DcMotorEx leftMotorLeft;
    private DcMotorEx leftMotorRight;
    private DcMotorEx rightMotorLeft;
    private DcMotorEx rightMotorRight;
    private AnalogInput leftAbsolute;
    private AnalogInput rightAbsolute;
    private final SwervePodEncoder leftTracker = new SwervePodEncoder(SwervePodEncoder.LEFT_QUADRATURE_SIGN);
    private final SwervePodEncoder rightTracker = new SwervePodEncoder(SwervePodEncoder.RIGHT_QUADRATURE_SIGN);
    private List<LynxModule> hubs;
    private LynxModule.BulkCachingMode[] previousModes;
    private int leftZero;
    private int rightZero;
    private boolean zeroCaptured;
    private boolean selectedLeft = true;
    private boolean revolutionActive;
    private int revolutionStartCount;
    private String revolutionResult = "not measured";
    private boolean aligning;
    private long lastAlignmentNanos;
    private PodAlignmentController alignment;
    private boolean wasA;
    private boolean wasB;
    private boolean wasX;
    private boolean wasY;
    private boolean wasLeftBumper;
    private boolean wasRightBumper;
    private long lastSample;

    @Override
    public void init() {
        encoderleft = hardwareMap.get(DcMotor.class, "encoderleft");
        encoderright = hardwareMap.get(DcMotor.class, "encoderright");
        leftAbsolute = hardwareMap.get(AnalogInput.class, SwervePodEncoder.LEFT_ANALOG_NAME);
        rightAbsolute = hardwareMap.get(AnalogInput.class, SwervePodEncoder.RIGHT_ANALOG_NAME);
        leftMotorLeft = getDriveMotor("motor0");
        leftMotorRight = getDriveMotor("motor1");
        rightMotorLeft = getDriveMotor("motor2");
        rightMotorRight = getDriveMotor("motor3");
        if (encoderleft.getPortNumber() != 0 || encoderright.getPortNumber() != 1
                || encoderleft.getController() != encoderright.getController()) {
            throw new IllegalArgumentException(
                    "Configure encoderleft/encoderright on Expansion Hub motor channels 0/1");
        }
        for (DcMotorEx motor : new DcMotorEx[]{leftMotorLeft, leftMotorRight, rightMotorLeft, rightMotorRight}) {
            if (motor.getController() == encoderleft.getController()) {
                throw new IllegalArgumentException("Pod encoders must be on a separate hub from drive motors");
            }
        }
        hubs = hardwareMap.getAll(LynxModule.class);
        previousModes = new LynxModule.BulkCachingMode[hubs.size()];
        for (int i = 0; i < hubs.size(); i++) {
            previousModes[i] = hubs.get(i).getBulkCachingMode();
            hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.setMsTransmissionInterval(100);
    }

    private DcMotorEx getDriveMotor(String name) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    @Override
    public void init_loop() {
        loop();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            if (hub.getBulkData().isFake()) {
                stopAllMotors();
                telemetry.addLine("Encoder read failed; no measurement this cycle.");
                telemetry.update();
                return;
            }
        }

        int leftCount = encoderleft.getController().getMotorCurrentPosition(encoderleft.getPortNumber());
        int rightCount = encoderright.getController().getMotorCurrentPosition(encoderright.getPortNumber());
        double leftVolts = leftAbsolute.getVoltage();
        double rightVolts = rightAbsolute.getVoltage();
        boolean analogValid = SwervePodEncoder.validVoltage(leftVolts)
                && SwervePodEncoder.validVoltage(rightVolts);
        long now = System.nanoTime();

        if (!zeroCaptured) {
            leftZero = leftCount;
            rightZero = rightCount;
            zeroCaptured = true;
            leftTracker.seed(0.0, leftCount);
            rightTracker.seed(0.0, rightCount);
            lastSample = now;
        }

        handleButtons(leftCount, rightCount, analogValid);
        boolean wasAligning = aligning;
        if (aligning) updateAlignment(leftVolts, rightVolts);
        else stopAllMotors();

        if (wasAligning && !aligning && alignment != null && alignment.isComplete()) {
            if (selectedLeft) {
                leftZero = leftCount;
                leftTracker.seed(0.0, leftCount);
            } else {
                rightZero = rightCount;
                rightTracker.seed(0.0, rightCount);
            }
        } else if (!aligning) {
            double seconds = Math.max(0.000001, (now - lastSample) * 1e-9);
            leftTracker.update(leftCount, seconds);
            rightTracker.update(rightCount, seconds);
        }
        lastSample = now;

        int leftDelta = leftCount - leftZero;
        int rightDelta = rightCount - rightZero;
        telemetry.addLine("Swerve encoder commissioning diagnostic.");
        telemetry.addLine("LB/RB select pod; A zeroes selected quadrature; X measures one revolution.");
        telemetry.addLine("Y steers selected pod to analog forward reference; B aborts alignment.");
        telemetry.addLine("Quadrature zero is software-only; no hardware counter reset is sent.");
        telemetry.addData("Selected pod", selectedLeft ? "LEFT (LB)" : "RIGHT (RB)");
        telemetry.addData("Analog valid", analogValid);
        telemetry.addData("Drive calibration verified", SwervePodEncoder.calibrationReady());
        telemetry.addData("Specified counts/revolution", SwervePodEncoder.COUNTS_PER_REVOLUTION);
        telemetry.addData("Left raw / from zero", "%d / %d", leftCount, leftDelta);
        telemetry.addData("Right raw / from zero", "%d / %d", rightCount, rightDelta);
        telemetry.addData("Left quadrature angle (deg)", Math.toDegrees(leftTracker.getAngleRadians()));
        telemetry.addData("Right quadrature angle (deg)", Math.toDegrees(rightTracker.getAngleRadians()));
        telemetry.addData("360 measurement", revolutionActive ? "ACTIVE: rotate selected pod, press X" : revolutionResult);
        telemetry.addData("Alignment", aligning ? "ACTIVE - B abort" : "idle");
        if (alignment != null) telemetry.addData("Alignment status", alignment.getStatus());
        showAnalog("Left", leftAbsolute, leftVolts, true);
        showAnalog("Right", rightAbsolute, rightVolts, false);
        telemetry.update();
    }

    private void handleButtons(int leftCount, int rightCount, boolean analogValid) {
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        if (leftBumper && !wasLeftBumper) selectPod(true);
        if (rightBumper && !wasRightBumper) selectPod(false);
        if (a && !wasA) zeroSelected(leftCount, rightCount);
        if (x && !wasX) toggleRevolution(leftCount);
        if (y && !wasY && analogValid) startAlignment();
        if (b && !wasB) stopAlignment("operator abort");
        wasLeftBumper = leftBumper;
        wasRightBumper = rightBumper;
        wasA = a;
        wasB = b;
        wasX = x;
        wasY = y;
    }

    private void selectPod(boolean left) {
        selectedLeft = left;
        if (aligning) stopAlignment("pod selection changed");
        revolutionActive = false;
        revolutionResult = "not measured";
    }

    private void zeroSelected(int leftCount, int rightCount) {
        if (selectedLeft) {
            leftZero = leftCount;
            leftTracker.seed(0.0, leftCount);
        } else {
            rightZero = rightCount;
            rightTracker.seed(0.0, rightCount);
        }
        revolutionResult = "selected quadrature zeroed";
    }

    private void toggleRevolution(int currentCount) {
        if (!revolutionActive) {
            revolutionActive = true;
            revolutionStartCount = currentCount;
            revolutionResult = "started at " + currentCount;
        } else {
            int delta = currentCount - revolutionStartCount;
            double expected = SwervePodEncoder.COUNTS_PER_REVOLUTION;
            double error = Math.abs(Math.abs(delta) - expected);
            revolutionActive = false;
            revolutionResult = String.format(java.util.Locale.US, "delta %d; abs error %.1f counts", delta, error);
        }
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

    private void showAnalog(String pod, AnalogInput input, double volts, boolean leftPod) {
        telemetry.addData(pod + " analog connection", input.getConnectionInfo());
        telemetry.addData(pod + " voltage/raw (V/deg)", "%.4f / %.3f",
                volts, SwervePodEncoder.validVoltage(volts) ? SwervePodEncoder.rawDegrees(volts) : Double.NaN);
        if (!SwervePodEncoder.validVoltage(volts)) return;
        double raw = SwervePodEncoder.rawDegrees(volts);
        double target = SwervePodEncoder.forwardTargetDegrees(leftPod);
        double error = SwervePodEncoder.analogErrorDegrees(volts, target,
                leftPod ? SwervePodEncoder.LEFT_ANALOG_SIGN : SwervePodEncoder.RIGHT_ANALOG_SIGN);
        telemetry.addData(pod + " target/error (deg)", "%.3f / %.3f", target, error);
        telemetry.addData(pod + " wrap CW/CCW (deg)", "%.1f / %.1f", 360.0 - raw, raw);
    }

    private void setSteeringVelocity(double steer, boolean leftPod) {
        double leftVelocity = steer * MOTOR_TICKS_PER_SECOND;
        if (leftPod) {
            leftMotorLeft.setVelocity(leftVelocity);
            leftMotorRight.setVelocity(-leftVelocity);
        } else {
            rightMotorLeft.setVelocity(leftVelocity);
            rightMotorRight.setVelocity(-leftVelocity);
        }
    }

    private void stopAlignment(String reason) {
        if (alignment != null && alignment.isActive()) alignment.abort(reason);
        aligning = false;
        stopAllMotors();
        revolutionResult = "alignment: " + reason;
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
        if (previousModes == null) return;
        for (int i = 0; i < hubs.size(); i++) {
            if (previousModes[i] != null) hubs.get(i).setBulkCachingMode(previousModes[i]);
        }
    }
}
