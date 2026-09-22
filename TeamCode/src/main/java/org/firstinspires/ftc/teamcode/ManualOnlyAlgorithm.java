package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathTracker;

import java.util.Collections;
import java.util.Map;

/** Allows Pedro manual mode before the measured Foresight model is approved. */
public final class ManualOnlyAlgorithm implements Algorithm {
    private IllegalStateException pathingDisabled() {
        return new IllegalStateException("Foresight pathing is disabled until characterization is verified");
    }
    @Override public DrivePowers calculatePath(Drivetrain drivetrain, PathTracker pathTracker,
                                                MotionState state, double deltaTime) {
        throw pathingDisabled();
    }
    @Override public DrivePowers calculateHold(Drivetrain drivetrain, Pose target,
                                                MotionState state, boolean useScaling, double deltaTime) {
        throw pathingDisabled();
    }
    @Override public double completion() { return 0.0; }
    @Override public Pose closestPose() { return Pose.zero(); }
    @Override public Vector2D closestTangent() { return Vector2D.zero(); }
    @Override public Vector2D closestNormal() { return Vector2D.zero(); }
    @Override public double curvature() { return 0.0; }
    @Override public double remainingDistance() { return 0.0; }
    @Override public boolean atParametricEnd() { return true; }
    @Override public void reset() { }
    @Override public boolean isBusy() { return false; }
    @Override public Map<String, Object> debug() { return Collections.singletonMap("mode", "manual-only"); }
}
