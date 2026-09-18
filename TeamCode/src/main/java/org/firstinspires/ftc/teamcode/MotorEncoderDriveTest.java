package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Motor direction and bounded differential pod commissioning diagnostic.
 * D-pad selects an individual motor; LB/RB select combined left/right pod control.
 */
@TeleOp(name = "Individual Motor Drive Test", group = "Differential Swerve")
public class MotorEncoderDriveTest extends OpMode {
    private static final double MAX_TEST_POWER = HardwareConstants.MAX_TEST_POWER;
    private static final double MAX_COMBINED_TICKS_PER_SECOND = HardwareConstants.MAX_COMBINED_TEST_TICKS_PER_SECOND;

    private DcMotorEx[] motors;
    private String[] names;
    private int selectedMotor;

    @Override
    public void init() {
        names = new String[]{
                HardwareConstants.MOTOR_LEFT_POD_LEFT,
                HardwareConstants.MOTOR_LEFT_POD_RIGHT,
                HardwareConstants.MOTOR_RIGHT_POD_LEFT,
                HardwareConstants.MOTOR_RIGHT_POD_RIGHT
        };
        motors = new DcMotorEx[names.length];
        for (int i = 0; i < names.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
            motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        telemetry.setMsTransmissionInterval(100);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            driveCombinedPod(0, 1, "LEFT");
        } else if (gamepad1.right_bumper) {
            driveCombinedPod(2, 3, "RIGHT");
        } else {
            driveIndividualMotor();
        }

        displayTelemetry();
        telemetry.update();
    }

    private void driveIndividualMotor() {
        selectMotor();
        double power = clamp(-gamepad1.left_stick_y, -MAX_TEST_POWER, MAX_TEST_POWER);
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(i == selectedMotor ? power : 0.0);
        }
        telemetry.addData("Mode", "INDIVIDUAL MOTOR");
        telemetry.addData("Selected motor", names[selectedMotor]);
        telemetry.addData("Command", "%.3f (left stick Y)", power);
        telemetry.addLine("D-pad: Up=motor0, Right=motor1, Down=motor2, Left=motor3");
    }

    private void driveCombinedPod(int leftMotorIndex, int rightMotorIndex, String podName) {
        double drive = clamp(-gamepad1.left_stick_y, -1.0, 1.0);
        // Positive operator input should turn the pod right/clockwise.
        double steer = DifferentialSwervePodController.motorSteeringForClockwise(
                clamp(gamepad1.right_stick_x, -1.0, 1.0));
        double leftCommand = drive + steer;
        double rightCommand = drive - steer;
        double scale = Math.max(1.0, Math.max(Math.abs(leftCommand), Math.abs(rightCommand)));
        leftCommand /= scale;
        rightCommand /= scale;

        stopAllMotors();
        motors[leftMotorIndex].setVelocity(leftCommand * MAX_COMBINED_TICKS_PER_SECOND);
        motors[rightMotorIndex].setVelocity(rightCommand * MAX_COMBINED_TICKS_PER_SECOND);

        telemetry.addData("Mode", "COMBINED %s POD", podName);
        telemetry.addData("Drive", "%.2f (left stick Y)", drive);
        telemetry.addData("Steer", "%.2f (right stick X, right=clockwise)", steer);
        telemetry.addData("Mixed motor commands", "left %.2f / right %.2f", leftCommand, rightCommand);
        telemetry.addLine("LB=left pod, RB=right pod; release bumper to return to individual mode.");
        telemetry.addLine("Combined output capped at 420 ticks/s; robot must remain raised.");
    }

    private void selectMotor() {
        if (gamepad1.dpad_up) selectedMotor = 0;
        else if (gamepad1.dpad_right) selectedMotor = 1;
        else if (gamepad1.dpad_down) selectedMotor = 2;
        else if (gamepad1.dpad_left) selectedMotor = 3;
    }

    private void displayTelemetry() {
        telemetry.addLine("CAUTION: robot raised; only the selected motor or pod is powered.");
        telemetry.addLine("Release sticks before changing mode or selection.");
        for (int i = 0; i < motors.length; i++) {
            telemetry.addData(names[i], "port %d | power %.2f | count %d | velocity %.1f",
                    motors[i].getPortNumber(), motors[i].getPower(),
                    motors[i].getCurrentPosition(), motors[i].getVelocity());
        }
        telemetry.addData("Pod quadrature", "left %d | right %d",
                getPodCount("encoderleft"), getPodCount("encoderright"));
        telemetry.addData("Differential logic", "left motor = drive + steer; right motor = drive - steer");
    }

    private int getPodCount(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name).getCurrentPosition();
        } catch (RuntimeException ignored) {
            return 0;
        }
    }

    private void stopAllMotors() {
        for (DcMotorEx motor : motors) motor.setVelocity(0.0);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        if (motors != null) stopAllMotors();
    }
}
