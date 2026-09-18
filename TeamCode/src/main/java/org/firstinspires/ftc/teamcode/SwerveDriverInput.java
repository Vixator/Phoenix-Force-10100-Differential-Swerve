package org.firstinspires.ftc.teamcode;

/** Converts FTC stick axes to normalized robot-forward/right and CW radians/second. */
public final class SwerveDriverInput {
    public static final double DRIVE_DEADBAND = 0.05;
    public static final double TURN_DEADBAND = 0.05;
    // At full stick, pure rotation requests full forward on one wheel and full
    // reverse on the other: omega = wheel speed / half the pod spacing.
    public static final double MAX_TURN_RATE = 2.0 * HardwareConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND
            / HardwareConstants.TRACK_WIDTH_METERS;

    private double forward;
    private double strafe;
    private double turn;

    public void update(double leftX, double leftY, double rightX) {
        double magnitude = Math.hypot(leftX, leftY);
        double speed = deadband(magnitude, DRIVE_DEADBAND);
        double scale = magnitude > 0.0 ? speed / magnitude : 0.0;
        forward = -leftY * scale;
        strafe = leftX * scale;
        double rotation = deadband(rightX, TURN_DEADBAND);
        turn = rotation * rotation * rotation * MAX_TURN_RATE;
    }

    private static double deadband(double value, double threshold) {
        double magnitude = Math.min(1.0, Math.abs(value));
        return magnitude <= threshold ? 0.0
                : Math.copySign((magnitude - threshold) / (1.0 - threshold), value);
    }

    public double getForward() { return forward; }
    public double getStrafe() { return strafe; }
    public double getTurn() { return turn; }
}
