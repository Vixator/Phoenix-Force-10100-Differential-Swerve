package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Read-only diagnostic for the raw goBILDA Pinpoint encoder counts.
 *
 * <p>Configure the Pinpoint as {@code pinpoint} on Control Hub I2C port 1. This OpMode does not
 * configure, reset, or change the direction of either Pinpoint encoder, and it never commands a
 * drivetrain motor. Use it to determine the physical signs before setting encoder directions in a
 * future Pinpoint localizer.</p>
 */
@TeleOp(name = "Pinpoint Encoder Direction Test", group = "Differential Swerve")
public class PinpointEncoderDirectionTest extends OpMode {
    private GoBildaPinpointDriver pinpoint;
    private int xZero;
    private int yZero;
    private boolean wasA;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PinpointSettings.NAME);
        pinpoint.update();
        captureZero();
        telemetry.addLine("Pinpoint found. No drivetrain motors are commanded.");
        telemetry.addLine("Keep the robot stationary until Start.");
        telemetry.update();
    }

    @Override
    public void loop() {
        pinpoint.update();

        boolean a = gamepad1.a;
        if (a && !wasA) captureZero();
        wasA = a;

        int x = pinpoint.getEncoderX();
        int y = pinpoint.getEncoderY();

        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Candidate X / Y directions", "%s / %s",
                PinpointSettings.X_DIRECTION_CANDIDATE, PinpointSettings.Y_DIRECTION_CANDIDATE);
        telemetry.addData("Directions verified", PinpointSettings.DIRECTIONS_VERIFIED);
        telemetry.addData("Pinpoint loop / frequency", "%d us / %.0f Hz",
                pinpoint.getLoopTime(), pinpoint.getFrequency());
        telemetry.addData("X encoder raw / delta", "%d / %+d", x, x - xZero);
        telemetry.addData("Y encoder raw / delta", "%d / %+d", y, y - yZero);
        telemetry.addLine("A: set the displayed deltas to zero.");
        telemetry.addLine("Push robot forward: X delta should increase.");
        telemetry.addLine("Move robot left: Y delta should increase.");
        telemetry.addLine("If a direction is opposite, record it; do not change it in this test.");
        telemetry.update();
    }

    private void captureZero() {
        pinpoint.update();
        xZero = pinpoint.getEncoderX();
        yZero = pinpoint.getEncoderY();
    }
}
