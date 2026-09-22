package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Locale;

/** Captures bounded speed, BRAKE deceleration, rotation, and battery observations. */
@TeleOp(name = "Pedro Drive Characterization", group = "Pedro Commissioning")
public final class PedroDriveCharacterizationTest extends LinearOpMode {
    @Override public void runOpMode() {
        PedroAutoDrive drive = new PedroAutoDrive(this);
        try {
            if (!drive.initialize()) return;
            telemetry.addLine("D-pad = +/- forward/left; bumpers = +/- CCW. Release for BRAKE data.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            drive.arm(Pose.zero());
            Follower follower = drive.follower();
            long started = System.nanoTime();
            long nextLog = started;
            long nextTelemetry = started;
            RobotLog.ii("PedroCharacterization", "seconds,forward,left,ccw,x_in,y_in,heading_rad,vx_body_in_s,vy_body_in_s,omega_rad_s,battery_v,drive_scale");
            double peakForward = 0.0;
            double peakStrafe = 0.0;
            double peakOmega = 0.0;
            while (opModeIsActive() && !drive.hasFault()) {
                double forward = gamepad1.dpad_up ? 1.0 : gamepad1.dpad_down ? -1.0 : 0.0;
                double left = gamepad1.dpad_left ? 1.0 : gamepad1.dpad_right ? -1.0 : 0.0;
                double ccw = gamepad1.left_bumper ? PedroDriveConfig.AUTONOMOUS_TURN_LIMIT
                        : gamepad1.right_bumper ? -PedroDriveConfig.AUTONOMOUS_TURN_LIMIT : 0.0;
                follower.manual(forward, left, ccw);
                follower.update();
                Twist body = follower.twist();
                peakForward = Math.max(peakForward, Math.abs(body.vx));
                peakStrafe = Math.max(peakStrafe, Math.abs(body.vy));
                peakOmega = Math.max(peakOmega, Math.abs(body.omega));
                long now = System.nanoTime();
                if (now >= nextLog) {
                    nextLog = now + 20_000_000L;
                    Pose pose = follower.pose();
                    RobotLog.ii("PedroCharacterization", String.format(Locale.US,
                            "%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f",
                            (now - started) * 1e-9, forward, left, ccw,
                            pose.x(), pose.y(), pose.heading(), body.vx, body.vy, body.omega,
                            batteryVoltage(), PedroDriveConfig.AUTONOMOUS_MAX_DRIVE));
                }
                if (now < nextTelemetry) { idle(); continue; }
                nextTelemetry = now + 100_000_000L;
                telemetry.addLine("Signed samples recorded to Robot Controller log: PedroCharacterization");
                telemetry.addData("Command fwd / left / CCW", "%.2f / %.2f / %.2f", forward, left, ccw);
                telemetry.addData("Robot velocity in/s / in/s / rad/s", body);
                telemetry.addData("Peak |vx| / |vy| / |omega|", "%.3f / %.3f / %.3f",
                        peakForward, peakStrafe, peakOmega);
                telemetry.addData("Battery (V)", "%.2f", batteryVoltage());
                telemetry.addData("Pose", follower.pose());
                telemetry.update();
                idle();
            }
        } finally {
            drive.close();
        }
    }

    private double batteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0.0) result = Math.min(result, voltage);
        }
        return Double.isFinite(result) ? result : Double.NaN;
    }
}
