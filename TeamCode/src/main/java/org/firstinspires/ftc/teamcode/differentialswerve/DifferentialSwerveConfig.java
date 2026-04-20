package org.firstinspires.ftc.teamcode.differentialswerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class DifferentialSwerveConfig {

    private DifferentialSwerveConfig() {
    }

    public static final String LEFT_MOTOR_A = "leftMotorA";
    public static final String LEFT_MOTOR_B = "leftMotorB";
    public static final String RIGHT_MOTOR_A = "rightMotorA";
    public static final String RIGHT_MOTOR_B = "rightMotorB";

    public static final DcMotorSimple.Direction LEFT_MOTOR_A_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction LEFT_MOTOR_B_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_MOTOR_A_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_MOTOR_B_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;

    public static final double TRACK_WIDTH_METERS = 0.30;

    public static final double MOTOR_FREE_SPEED_RPM = 6000.0;
    public static final double FIRST_STAGE_DRIVE_RATIO = 24.0 / 60.0;
    public static final double BEVEL_TO_WHEEL_RATIO = 19.0 / 55.0;
    public static final double TOTAL_WHEEL_RATIO = FIRST_STAGE_DRIVE_RATIO * BEVEL_TO_WHEEL_RATIO;
    public static final double WHEEL_FREE_SPEED_RPM = MOTOR_FREE_SPEED_RPM * TOTAL_WHEEL_RATIO;

    public static final double STEERING_KP = 1.8;

    public static final double MAX_DRIVE_POWER = 0.85;
    public static final double MAX_STEER_POWER = 0.65;
    public static final double INPUT_DEADBAND = 0.08;
    public static final double TURN_INPUT_SCALE = 0.75;

    /*
     * This is the most important calibration value in the project.
     *
     * It converts encoder difference into pod angle:
     * podAngle += (deltaMotorA - deltaMotorB) * STEERING_RADIANS_PER_ENCODER_TICK
     *
     * Replace this placeholder with your real azimuth gearing once you know the steering path.
     * The wheel reduction above does not determine this value by itself.
     */
    public static final double STEERING_RADIANS_PER_ENCODER_TICK = (2.0 * Math.PI) / 5600.0;
}
