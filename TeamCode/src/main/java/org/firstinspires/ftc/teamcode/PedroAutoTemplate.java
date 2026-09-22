package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Normal Pedro 3.0.1 usage. Enable only after all commissioning gates are recorded and passed. */
@Disabled
@Autonomous(name = "Pedro Line Example", group = "Pedro Swerve")
public final class PedroAutoTemplate extends LinearOpMode {
    @Override public void runOpMode() {
        PedroAutoDrive drive = new PedroAutoDrive(this);
        try {
            if (!drive.initialize()) return;
            Follower follower = drive.follower();
            Pose start = new Pose(0, 0, 0);
            Path line = Paths.line(start, new Pose(12, 0, 0)).constant(start.heading());
            follower.holdEnd.set(false);
            waitForStart();
            if (isStopRequested()) return;
            drive.enableForesight();
            drive.arm(start);
            follower.follow(line);
            long deadline = System.nanoTime() + 10_000_000_000L;
            while (opModeIsActive() && follower.following() && !drive.hasFault()) {
                if (System.nanoTime() >= deadline) {
                    drive.abort("Line path exceeded 10 seconds");
                    break;
                }
                follower.update();
                telemetry.addData("Pose", follower.pose());
                telemetry.update();
                idle();
            }
            String result = drive.hasFault() ? "FAULT" : !opModeIsActive() ? "OPERATOR_STOP"
                    : PedroPathResult.evaluate(line.endPose(), follower.pose(), follower.velocity());
            follower.stop();
            if ("END_OUTSIDE_TOLERANCE".equals(result) || "INVALID_TERMINAL_FEEDBACK".equals(result)) drive.abort(result);
            telemetry.addData("Completion reason", result);
            telemetry.addData("Fault", drive.fault());
            telemetry.update();
        } finally {
            drive.close();
        }
    }
}
