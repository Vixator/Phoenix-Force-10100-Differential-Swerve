package org.firstinspires.ftc.teamcode;

/**
 * Shared hardware constants for differential swerve drivetrain.
 * Eliminates duplication across OpModes and provides single source of truth.
 */
public final class HardwareConstants {
    
    // Prevent instantiation of utility class
    private HardwareConstants() {
        throw new UnsupportedOperationException("HardwareConstants is a utility class and cannot be instantiated");
    }
    
    // ==================== HARDWARE NAMES ====================
    // Control Hub motor ports
    public static final String MOTOR_LEFT_POD_LEFT = "motor0";
    public static final String MOTOR_LEFT_POD_RIGHT = "motor1";
    public static final String MOTOR_RIGHT_POD_LEFT = "motor2";
    public static final String MOTOR_RIGHT_POD_RIGHT = "motor3";
    
    // Expansion Hub encoder ports (motor channels 0/1, read-only)
    public static final String ENCODER_LEFT = "encoderleft";
    public static final String ENCODER_RIGHT = "encoderright";
    
    // Control Hub analog ports (from SwervePodEncoder)
    public static final String ANALOG_LEFT = "absencleft";
    public static final String ANALOG_RIGHT = "absencright";
    
    // ==================== MOTOR SPECIFICATIONS ====================
    public static final double MOTOR_FREE_SPEED_RPM = 1150.0;
    public static final double MOTOR_TICKS_PER_REVOLUTION = 145.1;
    public static final double MAX_MOTOR_TICKS_PER_SECOND =
            MOTOR_FREE_SPEED_RPM * MOTOR_TICKS_PER_REVOLUTION / 60.0; // 2781.0833
    
    // ==================== DRIVETRAIN GEOMETRY ====================
    public static final double FIRST_STAGE_RATIO = 16.0 / 54.0;
    public static final double SECOND_STAGE_RATIO = 50.0 / 19.0;
    public static final double TOTAL_DRIVE_RATIO = FIRST_STAGE_RATIO * SECOND_STAGE_RATIO;
    public static final double WHEEL_DIAMETER_METERS = 0.06325; // 63.25 mm
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
    public static final double TRACK_WIDTH_METERS = 0.3595;
    public static final double TRACK_WIDTH_INCHES = TRACK_WIDTH_METERS / 0.0254;
    public static final double POD_CENTER_OFFSET_INCHES = TRACK_WIDTH_INCHES / 2.0;
    public static final double MAX_WHEEL_SPEED_METERS_PER_SECOND =
            MOTOR_FREE_SPEED_RPM / 60.0 * TOTAL_DRIVE_RATIO * WHEEL_CIRCUMFERENCE_METERS;
    
    // ==================== DIAGNOSTIC LIMITS ====================
    // Safe testing speeds for commissioning
    public static final double MAX_TEST_POWER = 0.25; // For MotorEncoderDriveTest individual mode
    public static final double MAX_COMBINED_TEST_TICKS_PER_SECOND = 420.0; // For pod mode
}
