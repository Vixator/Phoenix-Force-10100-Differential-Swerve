package org.firstinspires.ftc.teamcode;

/** Robot-relative forward/right translation and clockwise-positive chassis rotation. */
public final class DifferentialSwerveKinematics {
    private double leftTargetAngle;
    private double rightTargetAngle;
    private double leftSpeed;
    private double rightSpeed;

    public void update(double forward, double strafe, double clockwiseRadiansPerSecond) {
        double turnSpeed = clockwiseRadiansPerSecond * HardwareConstants.TRACK_WIDTH_METERS
                / (2.0 * HardwareConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND);
        // Clockwise chassis motion moves the left wheel forward and the right backward.
        // The sign of an individual pod's steering motors does not change this geometry.
        double leftForward = forward + turnSpeed;
        double rightForward = forward - turnSpeed;
        leftSpeed = Math.hypot(leftForward, strafe);
        rightSpeed = Math.hypot(rightForward, strafe);
        double magnitude = Math.max(1.0, Math.max(leftSpeed, rightSpeed));
        leftSpeed /= magnitude;
        rightSpeed /= magnitude;
        // A stopped pod retains its previous travel axis rather than snapping forward.
        if (leftSpeed > 1e-6) leftTargetAngle = Math.atan2(strafe, leftForward);
        if (rightSpeed > 1e-6) rightTargetAngle = Math.atan2(strafe, rightForward);
    }

    public double getLeftTargetAngle() { return leftTargetAngle; }
    public double getRightTargetAngle() { return rightTargetAngle; }
    public double getLeftSpeed() { return leftSpeed; }
    public double getRightSpeed() { return rightSpeed; }
}
