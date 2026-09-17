package org.firstinspires.ftc.teamcode;

/** Shared calibration and absolute-seeded, quadrature-only azimuth tracking. */
public final class SwervePodEncoder {
    public static final String LEFT_ANALOG_NAME = "absenc";
    public static final String RIGHT_ANALOG_NAME = "absenc2";
    public static final double FULL_SCALE_VOLTS = 3.2;

    // Melonbotics specifies 1024 CPR quadrature output. Verify the raw hub count
    // convention over one complete pod revolution during physical commissioning.
    public static final double COUNTS_PER_REVOLUTION = 1024.0;
    public static final int LEFT_QUADRATURE_SIGN = 1;
    public static final int RIGHT_QUADRATURE_SIGN = 1;
    public static final int LEFT_ANALOG_SIGN = 1;
    public static final int RIGHT_ANALOG_SIGN = 1;
    public static final boolean CALIBRATION_VERIFIED = false;

    // Raw analog degrees with the directed wheel-travel axis facing ROBOT forward.
    // Independent references absorb the right module's 180-degree mounting rotation.
    // Do not add another 180 degrees or negate right feedback just because it is rotated.
    public static final double LEFT_FORWARD_DEGREES = Double.NaN;
    public static final double RIGHT_FORWARD_DEGREES = Double.NaN;

    private final int quadratureSign;
    private int lastCount;
    private double angleRadians;
    private double rateRadiansPerSecond;

    public SwervePodEncoder(int quadratureSign) {
        if (quadratureSign != 1 && quadratureSign != -1) {
            throw new IllegalArgumentException("Quadrature sign must be +1 or -1");
        }
        this.quadratureSign = quadratureSign;
    }

    public static boolean calibrationReady() {
        return CALIBRATION_VERIFIED && validForward(LEFT_FORWARD_DEGREES)
                && validForward(RIGHT_FORWARD_DEGREES);
    }

    static boolean validForward(double degrees) {
        return Double.isFinite(degrees) && degrees >= 0.0 && degrees < 360.0;
    }

    /** Returns the configured forward reference, or electrical zero for commissioning tools. */
    public static double forwardTargetDegrees(boolean leftPod) {
        double configured = leftPod ? LEFT_FORWARD_DEGREES : RIGHT_FORWARD_DEGREES;
        return validForward(configured) ? configured : 0.0;
    }

    public static boolean validVoltage(double volts) {
        return Double.isFinite(volts) && volts >= 0.0 && volts <= FULL_SCALE_VOLTS;
    }

    public static double rawDegrees(double volts) {
        if (!validVoltage(volts)) throw new IllegalArgumentException("Analog voltage outside 0..3.2 V");
        return volts / FULL_SCALE_VOLTS * 360.0;
    }

    public static double absoluteRadians(double volts, double forwardDegrees, int analogSign) {
        if (!validForward(forwardDegrees) || (analogSign != 1 && analogSign != -1)) {
            throw new IllegalArgumentException("Missing/invalid pod forward calibration or analog sign");
        }
        return wrapRadians(Math.toRadians((rawDegrees(volts) - forwardDegrees) * analogSign));
    }

    /** Signed shortest analog error in degrees, positive in the programmed steering direction. */
    public static double analogErrorDegrees(double volts, double forwardDegrees, int analogSign) {
        if (!validForward(forwardDegrees) || (analogSign != 1 && analogSign != -1)) {
            throw new IllegalArgumentException("Missing/invalid pod forward calibration or analog sign");
        }
        return wrapDegrees((forwardDegrees - rawDegrees(volts)) * analogSign);
    }

    public void seed(double absoluteRadians, int count) {
        if (!Double.isFinite(absoluteRadians)) throw new IllegalArgumentException("Invalid initial angle");
        angleRadians = wrapRadians(absoluteRadians);
        lastCount = count;
        rateRadiansPerSecond = 0.0;
    }

    public void update(int count, double seconds) {
        if (!Double.isFinite(seconds) || seconds <= 0.0) throw new IllegalArgumentException("Invalid sample time");
        // Integer subtraction before conversion preserves signed counter rollover.
        double delta = (count - lastCount) * quadratureSign * (2.0 * Math.PI / COUNTS_PER_REVOLUTION);
        lastCount = count;
        angleRadians = wrapRadians(angleRadians + delta);
        rateRadiansPerSecond = delta / seconds;
    }

    public int getCount() { return lastCount; }
    public double getAngleRadians() { return angleRadians; }
    public double getRateRadiansPerSecond() { return rateRadiansPerSecond; }

    public static double wrapDegrees(double angle) {
        angle %= 360.0;
        if (angle <= -180.0) angle += 360.0;
        if (angle > 180.0) angle -= 360.0;
        return angle;
    }

    public static double wrapRadians(double angle) {
        angle %= 2.0 * Math.PI;
        if (angle <= -Math.PI) angle += 2.0 * Math.PI;
        if (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }
}
