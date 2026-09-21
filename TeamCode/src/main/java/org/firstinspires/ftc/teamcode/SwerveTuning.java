package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/** Live drivetrain gains shown under SwerveTuning in FTC Dashboard. */
@Config
public final class SwerveTuning {
    private SwerveTuning() { }

    // REV RUN_USING_ENCODER velocity PIDF, shared by all four drive motors.
    public static volatile double MOTOR_VELOCITY_P = 15.0;
    public static volatile double MOTOR_VELOCITY_I = 0.5;
    public static volatile double MOTOR_VELOCITY_D = 0.5;
    public static volatile double MOTOR_VELOCITY_F =
            32767.0 / HardwareConstants.MAX_MOTOR_TICKS_PER_SECOND;

    // Runtime pod azimuth controller. Error is radians and rate is radians/second.
    public static volatile double STEERING_P = 0.5;
    public static volatile double STEERING_D = 0.01;
    public static volatile double STEERING_MAX_COMMAND = 0.20;
    public static volatile double STEERING_SLEW_RATE = 2.0;

    // Absolute-encoder forward alignment performed during INIT. Error is degrees.
    public static volatile double ALIGNMENT_P = 0.005;
    // Overcome static friction near the 2-degree settling boundary without
    // increasing the command at larger errors beyond ALIGNMENT_MAX_COMMAND.
    public static volatile double ALIGNMENT_MIN_COMMAND = 0.03;
    public static volatile double ALIGNMENT_MAX_COMMAND = 0.20;

    public static void validate() {
        requireNonnegative("MOTOR_VELOCITY_P", MOTOR_VELOCITY_P);
        requireNonnegative("MOTOR_VELOCITY_I", MOTOR_VELOCITY_I);
        requireNonnegative("MOTOR_VELOCITY_D", MOTOR_VELOCITY_D);
        requireNonnegative("MOTOR_VELOCITY_F", MOTOR_VELOCITY_F);
        requireNonnegative("STEERING_P", STEERING_P);
        requireNonnegative("STEERING_D", STEERING_D);
        requireRange("STEERING_MAX_COMMAND", STEERING_MAX_COMMAND, 0.0, 1.0);
        requireNonnegative("STEERING_SLEW_RATE", STEERING_SLEW_RATE);
        requireNonnegative("ALIGNMENT_P", ALIGNMENT_P);
        requireRange("ALIGNMENT_MIN_COMMAND", ALIGNMENT_MIN_COMMAND, 0.0, 1.0);
        requireRange("ALIGNMENT_MAX_COMMAND", ALIGNMENT_MAX_COMMAND, 0.0, 1.0);
        if (ALIGNMENT_MIN_COMMAND > ALIGNMENT_MAX_COMMAND) {
            throw new IllegalArgumentException("ALIGNMENT_MIN_COMMAND must not exceed ALIGNMENT_MAX_COMMAND");
        }
    }

    private static void requireNonnegative(String name, double value) {
        requireRange(name, value, 0.0, Double.MAX_VALUE);
    }

    private static void requireRange(String name, double value, double min, double max) {
        if (!Double.isFinite(value) || value < min || value > max) {
            throw new IllegalArgumentException(name + " must be finite and in " + min + ".." + max);
        }
    }
}
