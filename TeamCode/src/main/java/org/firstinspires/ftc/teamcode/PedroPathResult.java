package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/** Independent endpoint acceptance: Pedro 3.0.1 FOLLOW completion is only parametric. */
final class PedroPathResult {
    private PedroPathResult() { }
    static String evaluate(Pose target, Pose actual, Velocity velocity) {
        double distance = target.distance(actual);
        double heading = Math.abs(SwervePodEncoder.wrapRadians(target.heading() - actual.heading()));
        double speed = Math.hypot(velocity.vx, velocity.vy);
        if (!Double.isFinite(distance) || !Double.isFinite(heading) || !Double.isFinite(speed)
                || !Double.isFinite(velocity.omega)) return "INVALID_TERMINAL_FEEDBACK";
        if (distance > .5 || heading > Math.toRadians(3) || speed > 1.0
                || Math.abs(velocity.omega) > Math.toRadians(5)) return "END_OUTSIDE_TOLERANCE";
        return "END_WITHIN_TOLERANCE";
    }
}
