package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp(name = "Swerve Pod Encoder Test", group = "Differential Swerve")
public class SwervePodEncoderTest extends OpMode {
    private static final double COUNTS_PER_REVOLUTION = 8192.0;

    private DcMotor encoderleft;
    private DcMotor encoderright;
    private List<LynxModule> hubs;
    private LynxModule.BulkCachingMode[] previousModes;
    private int leftZero;
    private int rightZero;
    private boolean zeroCaptured;
    private boolean wasZeroPressed;

    @Override
    public void init() {
        // Quadrature inputs are accessed through their corresponding configured motor channels.
        // No motor power, velocity, direction, mode, or hardware encoder reset is commanded here.
        encoderleft = hardwareMap.get(DcMotor.class, "encoderleft");
        encoderright = hardwareMap.get(DcMotor.class, "encoderright");
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
        boolean zeroPressed = gamepad1.a;
        boolean captureZero = zeroPressed && !wasZeroPressed;
        wasZeroPressed = zeroPressed;
        if (!zeroCaptured || captureZero) {
            leftZero = leftCount;
            rightZero = rightCount;
            zeroCaptured = true;
        }
        // Integer subtraction also handles a counter rollover during a short calibration movement.
        int leftDelta = leftCount - leftZero;
        int rightDelta = rightCount - rightZero;

        telemetry.addLine("READ-ONLY TEST: no motor outputs.");
        telemetry.addLine("Verify these channels belong to the Expansion Hub in Robot Configuration.");
        telemetry.addLine("Align both pods forward, then tap A to zero BOTH readings.");
        telemetry.addLine("Turn one POD 90 degrees CLOCKWISE viewed from above; expect about +2048 counts.");
        telemetry.addLine("Negative clockwise counts conflict with the drive TeleOp's calibrated polarity.");
        telemetry.addLine("Wheel rolling without pod rotation should NOT change these counts.");
        telemetry.addData("Left connection", encoderleft.getConnectionInfo());
        telemetry.addData("Right connection", encoderright.getConnectionInfo());
        telemetry.addData("Left raw counts", leftCount);
        telemetry.addData("Right raw counts", rightCount);
        telemetry.addData("Left counts from zero", leftDelta);
        telemetry.addData("Right counts from zero", rightDelta);
        telemetry.addData("Left angle from zero (deg)", leftDelta * 360.0 / COUNTS_PER_REVOLUTION);
        telemetry.addData("Right angle from zero (deg)", rightDelta * 360.0 / COUNTS_PER_REVOLUTION);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (previousModes == null) return;
        for (int i = 0; i < hubs.size(); i++) {
            if (previousModes[i] != null) hubs.get(i).setBulkCachingMode(previousModes[i]);
        }
    }
}
