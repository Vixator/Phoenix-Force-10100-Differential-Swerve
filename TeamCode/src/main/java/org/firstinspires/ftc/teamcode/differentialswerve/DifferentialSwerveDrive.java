package org.firstinspires.ftc.teamcode.differentialswerve;

public class DifferentialSwerveDrive {
    private final DifferentialSwervePod leftPod;
    private final DifferentialSwervePod rightPod;
    private final double trackWidthMeters;

    public DifferentialSwerveDrive(DifferentialSwervePod leftPod,
                                   DifferentialSwervePod rightPod,
                                   double trackWidthMeters) {
        this.leftPod = leftPod;
        this.rightPod = rightPod;
        this.trackWidthMeters = trackWidthMeters;
    }

    public void update() {
        leftPod.update();
        rightPod.update();
    }

    public void zeroPodsForward() {
        leftPod.setEstimatedAngleRadians(0.0);
        rightPod.setEstimatedAngleRadians(0.0);
    }

    public void driveRobotCentric(double forward,
                                  double strafeLeft,
                                  double turnCounterClockwise) {
        double halfTrack = trackWidthMeters / 2.0;

        // The two pods sit on the left and right sides of the chassis.
        double leftForward = forward - turnCounterClockwise * halfTrack;
        double leftStrafe = strafeLeft;
        double rightForward = forward + turnCounterClockwise * halfTrack;
        double rightStrafe = strafeLeft;

        double leftSpeed = Math.hypot(leftForward, leftStrafe);
        double rightSpeed = Math.hypot(rightForward, rightStrafe);
        double maxMagnitude = Math.max(1.0, Math.max(leftSpeed, rightSpeed));

        leftSpeed /= maxMagnitude;
        rightSpeed /= maxMagnitude;

        double leftAngle = Math.atan2(leftStrafe, leftForward);
        double rightAngle = Math.atan2(rightStrafe, rightForward);

        leftPod.setDesiredState(
                leftAngle,
                leftSpeed,
                DifferentialSwerveConfig.STEERING_KP,
                DifferentialSwerveConfig.MAX_DRIVE_POWER,
                DifferentialSwerveConfig.MAX_STEER_POWER
        );
        rightPod.setDesiredState(
                rightAngle,
                rightSpeed,
                DifferentialSwerveConfig.STEERING_KP,
                DifferentialSwerveConfig.MAX_DRIVE_POWER,
                DifferentialSwerveConfig.MAX_STEER_POWER
        );
    }

    public void stop() {
        leftPod.stop();
        rightPod.stop();
    }

    public DifferentialSwervePod getLeftPod() {
        return leftPod;
    }

    public DifferentialSwervePod getRightPod() {
        return rightPod;
    }
}
