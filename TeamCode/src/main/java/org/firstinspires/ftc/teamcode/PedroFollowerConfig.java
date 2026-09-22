package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

/** Complete Foresight configuration, gated on measured drivetrain characterization. */
public final class PedroFollowerConfig {
    public static final boolean MODEL_VERIFIED = false;
    public static final double CHARACTERIZED_DRIVE_SCALE = 0.15;

    // Deliberate placeholders: replace with recorded fits, then set MODEL_VERIFIED true.
    public static final double MAX_FORWARD_VELOCITY_INCHES_PER_SECOND = Double.NaN;
    public static final double MAX_STRAFE_VELOCITY_INCHES_PER_SECOND = Double.NaN;
    public static final double NATURAL_FORWARD_DECELERATION_INCHES_PER_SECOND_SQUARED = Double.NaN;
    public static final double NATURAL_STRAFE_DECELERATION_INCHES_PER_SECOND_SQUARED = Double.NaN;
    public static final double FORWARD_LINEAR_BRAKE = Double.NaN;
    public static final double STRAFE_LINEAR_BRAKE = Double.NaN;
    public static final double FORWARD_QUADRATIC_BRAKE = Double.NaN;
    public static final double STRAFE_QUADRATIC_BRAKE = Double.NaN;
    public static final double HEADING_LINEAR_BRAKE = Double.NaN;
    public static final double HEADING_QUADRATIC_BRAKE = Double.NaN;

    private PedroFollowerConfig() { }

    public static ForesightConfig create() {
        if (!MODEL_VERIFIED) {
            throw new IllegalStateException("Foresight model is unverified; complete PedroDriveCharacterizationTest first");
        }
        if (Math.abs(CHARACTERIZED_DRIVE_SCALE - PedroDriveConfig.AUTONOMOUS_MAX_DRIVE) > 1e-12) {
            throw new IllegalStateException("Drive scale changed after characterization");
        }
        return build(MAX_FORWARD_VELOCITY_INCHES_PER_SECOND,
                MAX_STRAFE_VELOCITY_INCHES_PER_SECOND,
                NATURAL_FORWARD_DECELERATION_INCHES_PER_SECOND_SQUARED,
                NATURAL_STRAFE_DECELERATION_INCHES_PER_SECOND_SQUARED,
                FORWARD_LINEAR_BRAKE, STRAFE_LINEAR_BRAKE,
                FORWARD_QUADRATIC_BRAKE, STRAFE_QUADRATIC_BRAKE,
                HEADING_LINEAR_BRAKE, HEADING_QUADRATIC_BRAKE);
    }

    static ForesightConfig syntheticForTests() {
        return build(12.0, 10.0, 9.0, 8.0,
                0.08, 0.09, 0.01, 0.012, 0.10, 0.02);
    }

    private static ForesightConfig build(double maxForward, double maxStrafe,
                                         double naturalForward, double naturalStrafe,
                                         double forwardLinear, double strafeLinear,
                                         double forwardQuadratic, double strafeQuadratic,
                                         double headingLinear, double headingQuadratic) {
        positive("maxForward", maxForward);
        positive("maxStrafe", maxStrafe);
        positive("naturalForward", naturalForward);
        positive("naturalStrafe", naturalStrafe);
        positive("forwardLinear", forwardLinear);
        positive("strafeLinear", strafeLinear);
        positive("forwardQuadratic", forwardQuadratic);
        positive("strafeQuadratic", strafeQuadratic);
        positive("headingLinear", headingLinear);
        positive("headingQuadratic", headingQuadratic);
        if (6.0 > Math.min(naturalForward, naturalStrafe)) {
            throw new IllegalArgumentException("Requested 6 in/s^2 deceleration exceeds measured natural deceleration");
        }

        ForesightConfig config = new ForesightConfig(c -> {
            c.headingFeedback.set(Controller.pid(0.5, 0.0, 0.0));
            c.headingStaticFF.set(Controller.staticFeedforward(0.0));
            c.forwardTranslational.set(Controller.pid(0.03, 0.0, 0.0));
            c.strafeTranslational.set(Controller.pid(0.03, 0.0, 0.0));
            c.brake.set(Controller.proportionalFeedforward(1.0 / Math.min(maxForward, maxStrafe)));
            c.coast.set(Controller.proportionalFeedforward(1.0 / Math.min(maxForward, maxStrafe)));
            c.holdPointTranslationalScaling.set(0.45);
            c.holdPointHeadingScaling.set(0.35);
            c.maxBrakingPower.set(0.20);
            c.maxAccelerationConstraint.set(8.0);
            c.maxVelocityConstraint.set(8.0);
            c.maxDecelerationConstraint.set(6.0);
            c.maxPathSpeed.set(ForesightConfig.Constraint.NONE);
            c.maxDecelerationScale.set(ForesightConfig.Constraint.NONE);
            c.brakeAggression.set(1.0);
            c.coastDownToVelocity.set(0.0);
            c.headingDeviationTolerance.set(Math.toRadians(11.25));
            c.translationalDeviationTolerance.set(2.5);
            c.brakeAtEnd.set(true);
            c.pathSkip.set(false);
            c.headingDriveRatio.set(0.5);
            c.linearBrakeCoefficients.set(Matrix.diag(forwardLinear, strafeLinear));
            c.quadraticBrakeCoefficients.set(Matrix.diag(forwardQuadratic, strafeQuadratic));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(headingLinear, headingQuadratic));
            c.cosineScale.set(false);
            c.maxAchievableForwardVelocity.set(maxForward);
            c.maxAchievableStrafeVelocity.set(maxStrafe);
            c.naturalForwardDeceleration.set(naturalForward);
            c.naturalStrafeDeceleration.set(naturalStrafe);
            c.minCorrectionDistance.set(1e-3);
            c.parametricTConstraint.set(0.025);
            c.headingConstraint.set(Math.toRadians(3.0));
            c.translationalConstraint.set(0.5);
            c.velocityConstraint.set(1.0);
            c.timeoutConstraint.set(1000.0); // milliseconds in Pedro 3.0.1
        });
        validateResolved(config);
        return config;
    }

    static void validateResolved(ForesightConfig c) {
        c.headingFeedback.get(); c.headingStaticFF.get(); c.forwardTranslational.get();
        c.strafeTranslational.get(); c.brake.get(); c.coast.get();
        requireFiniteMatrix(c.linearBrakeCoefficients.get(), "linearBrakeCoefficients");
        requireFiniteMatrix(c.quadraticBrakeCoefficients.get(), "quadraticBrakeCoefficients");
        Vector2D heading = c.headingBrakeCoefficients.get();
        positive("headingBrakeLinear", heading.x());
        positive("headingBrakeQuadratic", heading.y());
        positive("maxAchievableForwardVelocity", c.maxAchievableForwardVelocity.get());
        positive("maxAchievableStrafeVelocity", c.maxAchievableStrafeVelocity.get());
        positive("naturalForwardDeceleration", c.naturalForwardDeceleration.get());
        positive("naturalStrafeDeceleration", c.naturalStrafeDeceleration.get());
        positive("maxVelocityConstraint", c.maxVelocityConstraint.get());
        positive("maxAccelerationConstraint", c.maxAccelerationConstraint.get());
        positive("maxDecelerationConstraint", c.maxDecelerationConstraint.get());
        positive("timeoutConstraint", c.timeoutConstraint.get());
    }

    private static void requireFiniteMatrix(Matrix matrix, String name) {
        if (matrix.numRows() != 2 || matrix.numCols() != 2) {
            throw new IllegalArgumentException(name + " must be 2x2");
        }

        for (int row = 0; row < 2; row++) {
            for (int column = 0; column < 2; column++) {
                double value = matrix.get(row, column);
                if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " must be finite");
                if (row == column) positive(name + " diagonal", value);
                else if (value != 0.0) throw new IllegalArgumentException(name + " must be diagonal for the axis model");
            }
        }

    }

    private static void positive(String name, double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and positive");
        }
    }
}
