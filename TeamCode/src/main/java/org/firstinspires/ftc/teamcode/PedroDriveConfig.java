package org.firstinspires.ftc.teamcode;

/** Conservative, deliberately bounded settings for Pedro drivetrain commissioning. */
public final class PedroDriveConfig {
    public static final double AUTONOMOUS_MAX_DRIVE = 0.15;
    public static final double AUTONOMOUS_TURN_LIMIT = 0.20;
    public static final double MAX_COMBINED_MOTOR_COMMAND = 0.35;
    public static final double TELEOP_MAX_DRIVE = 1.0;
    public static final double TELEOP_TURN_LIMIT = 1.0;
    public static final double MAX_FEEDBACK_AGE_SECONDS = 0.250;
    public static final double PINPOINT_READY_TIMEOUT_SECONDS = 3.0;
    public static final boolean X_LOCK_ENABLED = false;
    public static final boolean VOLTAGE_COMPENSATION_ENABLED = false;

    private PedroDriveConfig() { }

    public static void validate() {
        finiteRange("AUTONOMOUS_MAX_DRIVE", AUTONOMOUS_MAX_DRIVE, 0.0, 1.0);
        finiteRange("AUTONOMOUS_TURN_LIMIT", AUTONOMOUS_TURN_LIMIT, 0.0, 1.0);
        finiteRange("MAX_COMBINED_MOTOR_COMMAND", MAX_COMBINED_MOTOR_COMMAND, 0.0, 1.0);
        finitePositive("MAX_FEEDBACK_AGE_SECONDS", MAX_FEEDBACK_AGE_SECONDS);
        finitePositive("PINPOINT_READY_TIMEOUT_SECONDS", PINPOINT_READY_TIMEOUT_SECONDS);
        if (X_LOCK_ENABLED) throw new IllegalArgumentException("Two-pod differential swerve has no planar X-lock");
        if (VOLTAGE_COMPENSATION_ENABLED) {
            throw new IllegalArgumentException("Voltage compensation is disabled until characterized");
        }
        if (!SwervePodEncoder.calibrationReady()) {
            throw new IllegalStateException("Pod azimuth calibration is not verified");
        }
        SwerveTuning.validate();
    }

    static void finitePositive(String name, double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and positive");
        }
    }

    static void finiteRange(String name, double value, double min, double max) {
        if (!Double.isFinite(value) || value < min || value > max) {
            throw new IllegalArgumentException(name + " must be finite and in " + min + ".." + max);
        }
    }
}
