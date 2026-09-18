package org.firstinspires.ftc.teamcode;

/** Hardware-independent, clockwise-positive pod control and differential motor mixing. */
public final class DifferentialSwervePodController {
    // Start with bounded steering comparable to the working analog alignment loop.
    public static final double DEFAULT_KP = 0.5;
    public static final double DEFAULT_KD = 0.01;
    public static final double DEFAULT_MAX_STEER = 0.20;
    public static final double DEFAULT_SLEW_RATE = 2.0;
    private static final double REVERSAL_HYSTERESIS = Math.toRadians(3.0);

    private boolean reversed;
    private boolean hasTarget;
    private double steeringCommand;
    private double angleError;
    private double optimizedTargetAngle;
    private double leftMotorCommand;
    private double rightMotorCommand;

    /**
     * The measured motor response is CCW for left-positive/right-negative.
     * Convert a CW-positive correction to the steering term in left=drive+steer,
     * right=drive-steer. Sensor signs and chassis rotation do not belong here.
     */
    public static double motorSteeringForClockwise(double clockwiseCommand) {
        return -clockwiseCommand;
    }

    public void update(double currentAngle, double measuredRate, double targetAngle,
                       double targetSpeed, double seconds, double kp, double kd,
                       double maxDrive, double maxSteer, double slewRate) {
        angleError = SwervePodEncoder.wrapRadians(targetAngle - currentAngle);
        if (!hasTarget) {
            reversed = Math.abs(angleError) > Math.PI / 2.0;
            hasTarget = true;
        } else if (Math.abs(angleError) > Math.PI / 2.0 + REVERSAL_HYSTERESIS) {
            reversed = true;
        } else if (Math.abs(angleError) < Math.PI / 2.0 - REVERSAL_HYSTERESIS) {
            reversed = false;
        }
        // Keep the chosen wheel direction through small encoder/stick noise near 90 degrees.
        if (reversed) {
            angleError = SwervePodEncoder.wrapRadians(angleError + Math.PI);
            targetSpeed = -targetSpeed;
        }
        optimizedTargetAngle = SwervePodEncoder.wrapRadians(currentAngle + angleError);

        // Both P and measured-rate damping are in the encoder's CW-positive frame.
        // Convert the ENTIRE PD correction, so D opposes rather than reinforces motion.
        double steerLimit = clamp(maxSteer, 0.0, 1.0);
        double requestedSteering = clamp(motorSteeringForClockwise(
                kp * angleError - kd * measuredRate), -steerLimit, steerLimit);
        if (slewRate > 0.0) {
            double step = slewRate * seconds;
            requestedSteering = clamp(requestedSteering,
                    steeringCommand - step, steeringCommand + step);
        }
        // A reduced limit must take effect even if the previous slew state was larger.
        steeringCommand = clamp(requestedSteering, -steerLimit, steerLimit);

        double alignment = Math.max(0.0, Math.cos(angleError));
        double drive = targetSpeed * alignment * alignment * clamp(maxDrive, 0.0, 1.0);
        double driveHeadroom = 1.0 - Math.abs(steeringCommand);
        drive = clamp(drive, -driveHeadroom, driveHeadroom);
        leftMotorCommand = drive + steeringCommand;
        rightMotorCommand = drive - steeringCommand;
        if (!Double.isFinite(leftMotorCommand) || !Double.isFinite(rightMotorCommand)) {
            throw new IllegalArgumentException("Nonfinite motor command; check tuning values");
        }
    }

    public double getAngleError() { return angleError; }

    /** Match software slew state to an externally commanded motor stop. Retain target choice. */
    public void stopOutput() {
        steeringCommand = 0.0;
        leftMotorCommand = 0.0;
        rightMotorCommand = 0.0;
    }
    public double getOptimizedTargetAngle() { return optimizedTargetAngle; }
    public double getSteeringCommand() { return steeringCommand; }
    public double getLeftMotorCommand() { return leftMotorCommand; }
    public double getRightMotorCommand() { return rightMotorCommand; }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
