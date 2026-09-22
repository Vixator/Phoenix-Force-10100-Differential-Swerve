package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Vector2D;

/** Pure coordinate, envelope, and interpolation helpers for the Pedro adapter. */
public final class PedroSwerveMath {
    private static final double EPSILON = 1e-12;

    private PedroSwerveMath() { }

    public static void requireFinite(DrivePowers powers, String label) {
        if (powers == null || !Double.isFinite(powers.forward())
                || !Double.isFinite(powers.strafe()) || !Double.isFinite(powers.turn())) {
            throw new IllegalArgumentException(label + " drive powers must be finite");
        }
    }

    /** Pedro uses robot-left and CCW; the established robot controller uses right and CW rad/s. */
    public static double rightFromPedroStrafe(double strafeLeft) {
        return -strafeLeft;
    }

    public static double clockwiseRadiansPerSecond(double normalizedCounterClockwiseTurn) {
        return -normalizedCounterClockwiseTurn * 2.0
                * HardwareConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND
                / HardwareConstants.TRACK_WIDTH_METERS;
    }

    /** Converts Pedro/generic-swerve wheel theta to the pod tracker's CW-positive forward-zero frame. */
    public static double encoderAngle(double wheelTheta) {
        return SwervePodEncoder.wrapRadians(Math.PI / 2.0 - wheelTheta);
    }

    public static double wheelAngle(double encoderAngle) {
        return SwervePodEncoder.wrapRadians(Math.PI / 2.0 - encoderAngle);
    }

    /** Unnormalized vectors in (forward, left), with normalized CCW turn. */
    public static Vector2D[] linearWheelVectors(DrivePowers powers) {
        requireFinite(powers, "wheel-vector");
        return new Vector2D[] {
                Vector2D.cartesian(powers.forward() - powers.turn(), powers.strafe()),
                Vector2D.cartesian(powers.forward() + powers.turn(), powers.strafe())
        };
    }

    /** Greatest lambda in [0,1] that keeps both wheel vectors and turn inside their envelopes. */
    public static double maxScaling(DrivePowers current, DrivePowers delta, double turnLimit) {
        requireFinite(current, "current");
        requireFinite(delta, "delta");
        PedroDriveConfig.finiteRange("turnLimit", turnLimit, 0.0, 1.0);

        Vector2D[] a = linearWheelVectors(current);
        Vector2D[] b = linearWheelVectors(delta);
        if (Math.abs(current.turn()) > turnLimit + EPSILON) {
            throw new IllegalArgumentException("Current turn is outside the configured envelope");
        }
        double lambda = 1.0;
        for (int i = 0; i < a.length; i++) {
            if (a[i].magnitudeSquared() > 1.0 + EPSILON) {
                throw new IllegalArgumentException("Current wheel vector is outside the unit envelope");
            }
            double quadratic = b[i].magnitudeSquared();
            if (quadratic == 0.0) continue;
            double linear = 2.0 * a[i].dot(b[i]);
            double constant = Math.min(0.0, a[i].magnitudeSquared() - 1.0);
            double discriminant = linear * linear - 4.0 * quadratic * constant;
            if (discriminant < -EPSILON) {
                throw new IllegalArgumentException("No feasible wheel-vector interval");
            }
            double root = Math.sqrt(Math.max(0.0, discriminant));
            // Avoid subtracting nearly equal numbers for a small outward delta at the boundary.
            double exit = linear > 0.0 ? -2.0 * constant / (linear + root)
                    : (-linear + root) / (2.0 * quadratic);
            lambda = Math.min(lambda, exit);
        }

        double turnDelta = delta.turn();
        if (turnDelta != 0.0) {
            double exit = turnDelta > 0.0
                    ? (turnLimit - current.turn()) / turnDelta
                    : (-turnLimit - current.turn()) / turnDelta;
            lambda = Math.min(lambda, exit);
        }
        if (!Double.isFinite(lambda)) throw new IllegalArgumentException("Nonfinite scaling result");
        return clamp(lambda, 0.0, 1.0);
    }

    public static double interpolateVelocity(double xRadius, double yRadius, double theta) {
        PedroDriveConfig.finitePositive("xRadius", xRadius);
        PedroDriveConfig.finitePositive("yRadius", yRadius);
        if (!Double.isFinite(theta)) throw new IllegalArgumentException("theta must be finite");
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double result = 1.0 / Math.sqrt(cos * cos / (xRadius * xRadius)
                + sin * sin / (yRadius * yRadius));
        if (!Double.isFinite(result) || result <= 0.0) {
            throw new IllegalArgumentException("Invalid interpolated velocity");
        }
        return result;
    }

    static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
