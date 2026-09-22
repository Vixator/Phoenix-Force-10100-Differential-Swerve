package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Shared deadline/no-progress envelope for the disabled purpose-specific path diagnostics. */
abstract class PedroPathTestBase extends LinearOpMode {
    protected abstract Pose startPose();
    protected abstract Path path();

    @Override public final void runOpMode() {
        PedroAutoDrive drive = new PedroAutoDrive(this);
        try {
            if (!drive.initialize()) return;
            Follower follower = drive.follower();
            follower.holdEnd.set(false);
            waitForStart();
            if (isStopRequested()) return;
            drive.enableForesight();
            drive.arm(startPose());
            Path requestedPath = path();
            follower.follow(requestedPath);

            long started = System.nanoTime();
            long deadline = started + 10_000_000_000L;
            long lastProgress = started;
            double bestDistance = Double.POSITIVE_INFINITY;
            double bestCompletion = -1.0;
            double bestHeadingError = Double.POSITIVE_INFINITY;
            while (opModeIsActive() && follower.following() && !drive.hasFault()) {
                long now = System.nanoTime();
                if (now >= deadline) {
                    drive.abort("Path exceeded the 10 second overall deadline");
                    break;
                }
                follower.update();
                double distance = requestedPath.endPose().distance(follower.pose());
                double headingError = Math.abs(SwervePodEncoder.wrapRadians(
                        requestedPath.endPose().heading() - follower.pose().heading()));
                double completion = follower.following() ? follower.completion() : 1.0;
                if (distance + 0.10 < bestDistance || completion > bestCompletion + .005
                        || headingError + Math.toRadians(1) < bestHeadingError) {
                    bestDistance = Math.min(bestDistance, distance);
                    bestCompletion = Math.max(bestCompletion, completion);
                    bestHeadingError = Math.min(bestHeadingError, headingError);
                    lastProgress = now;
                } else if (now - started > 1_000_000_000L && now - lastProgress > 2_000_000_000L) {
                    drive.abort("No path progress for 2 seconds after the steering allowance");
                    break;
                }
                telemetry.addData("Pose", follower.pose());
                telemetry.addData("Velocity", follower.velocity());
                telemetry.addData("Endpoint distance (in)", distance);
                telemetry.addData("Follower busy / parametric end", "%s / %s",
                        follower.isBusy(), follower.atParametricEnd());
                telemetry.update();
                idle();
            }
            String result = drive.hasFault() ? "FAULT" : !opModeIsActive() ? "OPERATOR_STOP"
                    : PedroPathResult.evaluate(requestedPath.endPose(), follower.pose(), follower.velocity());
            follower.stop();
            if ("END_OUTSIDE_TOLERANCE".equals(result) || "INVALID_TERMINAL_FEEDBACK".equals(result)) drive.abort(result);
            telemetry.addData("Completion reason", result);
            telemetry.addData("Terminal endpoint distance (in)", requestedPath.endPose().distance(follower.pose()));
            telemetry.addData("Terminal pose", follower.pose());
            telemetry.addData("Fault", drive.fault());
            telemetry.update();
            sleep(1000);
        } finally {
            drive.close();
        }
    }
}
