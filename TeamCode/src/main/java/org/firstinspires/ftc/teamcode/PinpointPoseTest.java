package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Displays the goBILDA Pinpoint V2 pose estimate for commissioning.
 *
 * <p>Configure the device as {@code pinpoint} on Control Hub I2C port 1. The X (forward) pod is
 * 199.25 mm left of robot center and the Y (strafe) pod is 88.0 mm forward of robot center. The
 * commissioned encoder directions are applied explicitly and reported for runtime review.</p>
 */
@TeleOp(name = "Pinpoint Pose Test", group = "Differential Swerve")
public class PinpointPoseTest extends OpMode {
    private GoBildaPinpointDriver pinpoint;
    private boolean wasA;
    private double resetHeading;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PinpointSettings.NAME);
        pinpoint.setOffsets(PinpointSettings.X_POD_OFFSET_MM,
                PinpointSettings.Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(PinpointSettings.X_DIRECTION_CANDIDATE,
                PinpointSettings.Y_DIRECTION_CANDIDATE);
        telemetry.addLine("Pinpoint configured with measured 4-bar pod offsets.");
        telemetry.addLine("Keep robot stationary through Start while the Pinpoint IMU calibrates.");
        telemetry.addLine("A: reset X/Y/heading and recalibrate the IMU while stationary.");
        telemetry.update();
    }

    @Override
    public void start() {
        resetPoseAndImu();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        if (a && !wasA) resetPoseAndImu();
        wasA = a;

        pinpoint.update();
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("X position (mm)", "%.1f", pinpoint.getPosX(DistanceUnit.MM));
        telemetry.addData("Y position (mm)", "%.1f", pinpoint.getPosY(DistanceUnit.MM));
        telemetry.addData("X / Y position (in)", "%.3f / %.3f",
                pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("X / Y velocity (in/s)", "%.3f / %.3f",
                pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH));
        telemetry.addData("Z heading (deg)", "%.1f", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Wrapped heading delta (deg)", "%.1f", wrapDegrees(
                pinpoint.getHeading(AngleUnit.DEGREES) - resetHeading));
        telemetry.addData("Heading velocity (deg/s)", "%.1f",
                pinpoint.getHeadingVelocity(
                        org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("X / Y encoder counts", "%d / %d",
                pinpoint.getEncoderX(), pinpoint.getEncoderY());
        telemetry.addData("Pinpoint frequency (Hz)", "%.0f", pinpoint.getFrequency());
        telemetry.addData("Candidate X / Y directions", "%s / %s",
                PinpointSettings.X_DIRECTION_CANDIDATE, PinpointSettings.Y_DIRECTION_CANDIDATE);
        telemetry.addData("Directions / heading verified", "%s / %s",
                PinpointSettings.DIRECTIONS_VERIFIED, PinpointSettings.HEADING_CONVENTION);
        telemetry.addLine("At +90 deg, pushing robot forward should increase field Y.");
        telemetry.addLine("A: reset pose and recalibrate IMU. Robot must be stationary.");
        telemetry.update();
    }

    private void resetPoseAndImu() {
        pinpoint.resetPosAndIMU();
        resetHeading = 0.0;
    }

    private static double wrapDegrees(double angle) {
        angle %= 360.0;
        if (angle <= -180.0) angle += 360.0;
        if (angle > 180.0) angle -= 360.0;
        return angle;
    }
}
