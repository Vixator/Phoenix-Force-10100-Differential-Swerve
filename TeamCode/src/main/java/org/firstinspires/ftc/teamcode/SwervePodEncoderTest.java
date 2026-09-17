package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp(name = "Swerve Pod Encoder Test", group = "Differential Swerve")
public class SwervePodEncoderTest extends OpMode {
    private DcMotor encoderleft;
    private DcMotor encoderright;
    private AnalogInput leftAbsolute;
    private AnalogInput rightAbsolute;
    private final SwervePodEncoder leftTracker = new SwervePodEncoder(SwervePodEncoder.LEFT_QUADRATURE_SIGN);
    private final SwervePodEncoder rightTracker = new SwervePodEncoder(SwervePodEncoder.RIGHT_QUADRATURE_SIGN);
    private List<LynxModule> hubs;
    private LynxModule.BulkCachingMode[] previousModes;
    private int leftZero;
    private int rightZero;
    private boolean zeroCaptured;
    private boolean wasZeroPressed;
    private double leftForwardVolts = Double.NaN;
    private double rightForwardVolts = Double.NaN;
    private long lastSample;

    @Override
    public void init() {
        // Quadrature inputs are accessed through their corresponding configured motor channels.
        // No motor power, velocity, direction, mode, or hardware encoder reset is commanded here.
        encoderleft = hardwareMap.get(DcMotor.class, "encoderleft");
        encoderright = hardwareMap.get(DcMotor.class, "encoderright");
        // One joiner cable carries two independent signals on Control Hub analog channels 0/1.
        leftAbsolute = hardwareMap.get(AnalogInput.class, SwervePodEncoder.LEFT_ANALOG_NAME);
        rightAbsolute = hardwareMap.get(AnalogInput.class, SwervePodEncoder.RIGHT_ANALOG_NAME);
        if (encoderleft.getPortNumber() != 0 || encoderright.getPortNumber() != 1
                || encoderleft.getController() != encoderright.getController()) {
            throw new IllegalArgumentException(
                    "Configure encoderleft/encoderright on Expansion Hub motor channels 0/1");
        }
        hubs = hardwareMap.getAll(LynxModule.class);
        previousModes = new LynxModule.BulkCachingMode[hubs.size()];
        for (int i = 0; i < hubs.size(); i++) {
            previousModes[i] = hubs.get(i).getBulkCachingMode();
            hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.setMsTransmissionInterval(100);
    }

    @Override
    public void init_loop() {
        loop();
    }

    @Override
    public void loop() {
        // Motor getters below reuse this validated snapshot in MANUAL caching mode.
        for (int i = 0; i < hubs.size(); i++) {
            if (hubs.get(i).getBulkData().isFake()) {
                telemetry.addLine("Encoder read failed. Check hub connection; no measurement this cycle.");
                telemetry.update();
                return;
            }
        }

        // Read controller counts directly so a motor's configured direction cannot hide encoder polarity.
        int leftCount = encoderleft.getController().getMotorCurrentPosition(encoderleft.getPortNumber());
        int rightCount = encoderright.getController().getMotorCurrentPosition(encoderright.getPortNumber());
        double leftVolts = leftAbsolute.getVoltage();
        double rightVolts = rightAbsolute.getVoltage();
        boolean analogValid = SwervePodEncoder.validVoltage(leftVolts) && SwervePodEncoder.validVoltage(rightVolts);
        long now = System.nanoTime();
        boolean zeroPressed = gamepad1.a;
        boolean captureZero = zeroPressed && !wasZeroPressed && analogValid;
        wasZeroPressed = zeroPressed;
        if (!zeroCaptured || captureZero) {
            leftZero = leftCount;
            rightZero = rightCount;
            zeroCaptured = true;
            leftTracker.seed(0.0, leftCount);
            rightTracker.seed(0.0, rightCount);
        } else {
            double seconds = Math.max(0.000001, (now - lastSample) * 1e-9);
            leftTracker.update(leftCount, seconds);
            rightTracker.update(rightCount, seconds);
        }
        lastSample = now;
        if (captureZero) {
            leftForwardVolts = leftVolts;
            rightForwardVolts = rightVolts;
        }
        // Integer subtraction also handles a counter rollover during a short calibration movement.
        int leftDelta = leftCount - leftZero;
        int rightDelta = rightCount - rightZero;

        telemetry.addLine("READ-ONLY TEST: no motor outputs.");
        telemetry.addLine("Control Hub analog 0=absenc (left), 1=absenc2 (right), ONE joiner cable.");
        telemetry.addLine("Move ONE pod to verify channel assignment. Quadrature: Expansion Hub 0/1.");
        telemetry.addLine("Align directed wheel travel ROBOT forward; tap A to capture BOTH candidates.");
        telemetry.addLine("A only captures this session; it does NOT save drive calibration.");
        telemetry.addLine("Right module is rotated 180, NOT mirrored; calibrate its own forward reading.");
        telemetry.addLine("Measure signed counts at CW 90 degrees and one full revolution; CPR is provisional.");
        telemetry.addLine("Wheel rolling without pod rotation should NOT change these counts.");
        telemetry.addData("Analog valid", analogValid);
        telemetry.addData("Drive calibration verified", SwervePodEncoder.calibrationReady());
        telemetry.addData("Provisional hub counts/revolution", SwervePodEncoder.COUNTS_PER_REVOLUTION);
        telemetry.addData("Left connection", encoderleft.getConnectionInfo());
        telemetry.addData("Right connection", encoderright.getConnectionInfo());
        telemetry.addData("Left raw counts", leftCount);
        telemetry.addData("Right raw counts", rightCount);
        telemetry.addData("Left counts from zero", leftDelta);
        telemetry.addData("Right counts from zero", rightDelta);
        telemetry.addData("Left quadrature delta (deg, provisional)",
                leftDelta * 360.0 / SwervePodEncoder.COUNTS_PER_REVOLUTION * SwervePodEncoder.LEFT_QUADRATURE_SIGN);
        telemetry.addData("Right quadrature delta (deg, provisional)",
                rightDelta * 360.0 / SwervePodEncoder.COUNTS_PER_REVOLUTION * SwervePodEncoder.RIGHT_QUADRATURE_SIGN);
        showAnalog("Left", leftAbsolute, leftVolts, leftForwardVolts, leftTracker,
                SwervePodEncoder.LEFT_ANALOG_SIGN, SwervePodEncoder.LEFT_FORWARD_DEGREES);
        showAnalog("Right", rightAbsolute, rightVolts, rightForwardVolts, rightTracker,
                SwervePodEncoder.RIGHT_ANALOG_SIGN, SwervePodEncoder.RIGHT_FORWARD_DEGREES);
        telemetry.update();
    }

    private void showAnalog(String pod, AnalogInput input, double volts, double forwardVolts,
                            SwervePodEncoder tracker, int analogSign, double configuredForward) {
        telemetry.addData(pod + " analog connection", input.getConnectionInfo());
        telemetry.addData(pod + " voltage (V)", "%.4f", volts);
        if (!SwervePodEncoder.validVoltage(volts)) return;
        double raw = SwervePodEncoder.rawDegrees(volts);
        telemetry.addData(pod + " raw absolute (deg)", "%.3f", raw);
        // Distance to the DAC wrap, not a physical steering limit or a sensor dead zone.
        double cwMargin = analogSign > 0 ? 360.0 - raw : raw;
        double ccwMargin = analogSign > 0 ? raw : 360.0 - raw;
        telemetry.addData(pod + " to wrap CW / CCW (deg, sign provisional)", "%.1f / %.1f", cwMargin, ccwMargin);
        if (Double.isFinite(configuredForward)) {
            telemetry.addData(pod + " configured robot angle (deg)", "%.3f",
                    Math.toDegrees(SwervePodEncoder.absoluteRadians(volts, configuredForward, analogSign)));
        }
        if (Double.isFinite(forwardVolts)) {
            double forward = SwervePodEncoder.rawDegrees(forwardVolts);
            telemetry.addData(pod + " CAPTURED forward V / deg", "%.4f / %.3f", forwardVolts, forward);
            telemetry.addData(pod + " captured minimum wrap margin (deg)", "%.1f", Math.min(forward, 360.0 - forward));
            double absolute = SwervePodEncoder.absoluteRadians(volts, forward % 360.0, analogSign);
            telemetry.addData(pod + " analog from captured forward (deg)", "%.3f", Math.toDegrees(absolute));
            telemetry.addData(pod + " quadrature tracked from capture (deg)", "%.3f", Math.toDegrees(tracker.getAngleRadians()));
            telemetry.addData(pod + " analog minus quadrature (deg)", "%.3f",
                    Math.toDegrees(SwervePodEncoder.wrapRadians(absolute - tracker.getAngleRadians())));
        }
    }

    @Override
    public void stop() {
        if (previousModes == null) return;
        for (int i = 0; i < hubs.size(); i++) {
            if (previousModes[i] != null) hubs.get(i).setBulkCachingMode(previousModes[i]);
        }
    }
}
