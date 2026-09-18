package org.firstinspires.ftc.teamcode;

/** Shared calibration and absolute-seeded, quadrature-only azimuth tracking. */
public final class SwervePodEncoder {
    public static final String LEFT_ANALOG_NAME = "absencleft";
    public static final String RIGHT_ANALOG_NAME = "absencright";
    public static final double FULL_SCALE_VOLTS = 3.2;
    // Nominal DAC scale is 3.2 V; tolerate readings up to its 3.3 V supply rail.
    // This is an acceptance margin, not a replacement angle scale.
    public static final double MAX_VALID_VOLTS = 3.3;

    // The encoder specifies 1024 CPR; FTC quadrature tracking uses four edges
    // per cycle, so the selected raw pod scale is 4096 counts per revolution.
    public static final double COUNTS_PER_REVOLUTION = 4096.0;
    public static final int LEFT_QUADRATURE_SIGN = 1;
    public static final int RIGHT_QUADRATURE_SIGN = 1;
    // Both encoders decrease in raw voltage while their pods rotate clockwise.
    public static final int LEFT_ANALOG_SIGN = -1;
    public static final int RIGHT_ANALOG_SIGN = -1;
    // The measured top-dead-center references are the robot-forward zero for each pod.
    // Voltage conversion: volts / 3.2 * 360 degrees.
    public static final double LEFT_FORWARD_DEGREES = 13.725;
    public static final double RIGHT_FORWARD_DEGREES = 29.025;
    // Hardware, encoder signs/scale, and combined pod steering behavior are verified.
    // Remaining drivetrain work is powered drive commissioning and tuning.
    public static final boolean CALIBRATION_VERIFIED = true;

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
        return Double.isFinite(volts) && volts >= 0.0 && volts <= MAX_VALID_VOLTS;
    }

    public static double rawDegrees(double volts) {
        if (!validVoltage(volts)) throw new IllegalArgumentException(voltageFault(volts));
        // Saturate at the wrap endpoint; overscale must not invent an angle beyond 360.
        return Math.min(volts, FULL_SCALE_VOLTS) / FULL_SCALE_VOLTS * 360.0;
    }

    public static String voltageFault(double volts) {
        return "invalid analog feedback: " + volts + " V (accepted 0.." + MAX_VALID_VOLTS + " V)";
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
