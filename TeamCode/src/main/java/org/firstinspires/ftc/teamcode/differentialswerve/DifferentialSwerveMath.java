package org.firstinspires.ftc.teamcode.differentialswerve;

public final class DifferentialSwerveMath {

    private DifferentialSwerveMath() {
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) <= deadband) {
            return 0.0;
        }

        double scaled = (Math.abs(value) - deadband) / (1.0 - deadband);
        return Math.copySign(scaled, value);
    }

    public static double shapeInput(double value) {
        return value * value * value;
    }

    public static double wrapAngleRadians(double angle) {
        while (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }

    public static OptimizedModuleState optimize(double targetAngleRadians,
                                                double targetSpeed,
                                                double currentAngleRadians) {
        double delta = wrapAngleRadians(targetAngleRadians - currentAngleRadians);
        if (Math.abs(delta) > Math.PI / 2.0) {
            targetAngleRadians = wrapAngleRadians(targetAngleRadians + Math.PI);
            targetSpeed = -targetSpeed;
            delta = wrapAngleRadians(targetAngleRadians - currentAngleRadians);
        }
        return new OptimizedModuleState(targetAngleRadians, targetSpeed, delta);
    }

    public static final class OptimizedModuleState {
        public final double targetAngleRadians;
        public final double targetSpeed;
        public final double angleErrorRadians;

        public OptimizedModuleState(double targetAngleRadians,
                                    double targetSpeed,
                                    double angleErrorRadians) {
            this.targetAngleRadians = targetAngleRadians;
            this.targetSpeed = targetSpeed;
            this.angleErrorRadians = angleErrorRadians;
        }
    }
}
