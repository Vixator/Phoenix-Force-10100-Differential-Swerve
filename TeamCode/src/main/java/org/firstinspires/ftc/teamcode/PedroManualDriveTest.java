package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Low-speed robot-centric Pedro manual-drive sign and release-to-zero validation. */
@TeleOp(name = "Pedro Manual Drive Test", group = "Pedro Commissioning")
public final class PedroManualDriveTest extends LinearOpMode {
    private static final double INPUT_LIMIT = 0.35;

    @Override public void runOpMode() {
        PedroAutoDrive drive = new PedroAutoDrive(this);
        try {
            if (!drive.initialize()) return;
            waitForStart();
            if (isStopRequested()) return;
            drive.arm(Pose.zero());
            Follower follower = drive.follower();
            while (opModeIsActive() && !drive.hasFault()) {
                double forward = clamp(-gamepad1.left_stick_y) * INPUT_LIMIT;
                double left = clamp(-gamepad1.left_stick_x) * INPUT_LIMIT;
                double ccw = clamp(-gamepad1.right_stick_x) * INPUT_LIMIT;
                follower.manual(forward, left, ccw);
                follower.update();
                telemetry.addData("Robot command fwd / left / CCW", "%.2f / %.2f / %.2f",
                        forward, left, ccw);
                telemetry.addData("Pose (in, in, rad)", follower.pose());
                telemetry.addData("Velocity", follower.velocity());
                telemetry.update();
            }
        } finally {
            drive.close();
        }
    }

    private static double clamp(double value) { return Math.max(-1.0, Math.min(1.0, value)); }
}
