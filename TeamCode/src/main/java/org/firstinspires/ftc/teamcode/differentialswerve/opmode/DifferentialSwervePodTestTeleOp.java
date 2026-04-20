package org.firstinspires.ftc.teamcode.differentialswerve.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.differentialswerve.DifferentialSwerveConfig;
import org.firstinspires.ftc.teamcode.differentialswerve.DifferentialSwerveMath;
import org.firstinspires.ftc.teamcode.differentialswerve.DifferentialSwervePod;

@TeleOp(name = "Differential Swerve Pod Test", group = "Differential Swerve")
public class DifferentialSwervePodTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        DifferentialSwervePod leftPod = new DifferentialSwervePod(
                "left",
                hardwareMap,
                DifferentialSwerveConfig.LEFT_MOTOR_A,
                DifferentialSwerveConfig.LEFT_MOTOR_B,
                DifferentialSwerveConfig.LEFT_MOTOR_A_DIRECTION,
                DifferentialSwerveConfig.LEFT_MOTOR_B_DIRECTION,
                DifferentialSwerveConfig.STEERING_RADIANS_PER_ENCODER_TICK
        );
        DifferentialSwervePod rightPod = new DifferentialSwervePod(
                "right",
                hardwareMap,
                DifferentialSwerveConfig.RIGHT_MOTOR_A,
                DifferentialSwerveConfig.RIGHT_MOTOR_B,
                DifferentialSwerveConfig.RIGHT_MOTOR_A_DIRECTION,
                DifferentialSwerveConfig.RIGHT_MOTOR_B_DIRECTION,
                DifferentialSwerveConfig.STEERING_RADIANS_PER_ENCODER_TICK
        );

        DifferentialSwervePod activePod = leftPod;
        boolean previousX = false;
        boolean previousB = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x && !previousX) {
                activePod = leftPod;
                activePod.setEstimatedAngleRadians(0.0);
            }
            if (gamepad1.b && !previousB) {
                activePod = rightPod;
                activePod.setEstimatedAngleRadians(0.0);
            }
            previousX = gamepad1.x;
            previousB = gamepad1.b;

            leftPod.update();
            rightPod.update();

            double drivePower = -shapeAxis(gamepad1.left_stick_y) * 0.5;
            double steerPower = shapeAxis(gamepad1.right_stick_x) * 0.4;

            activePod.setRawDriveAndSteer(drivePower, steerPower);

            if (activePod != leftPod) {
                leftPod.stop();
            }
            if (activePod != rightPod) {
                rightPod.stop();
            }

            telemetry.addLine("X selects left pod, B selects right pod");
            telemetry.addLine("Left stick Y = wheel thrust, right stick X = steer");
            telemetry.addData("Active pod", activePod.getName());
            telemetry.addData("Left angle (deg)", Math.toDegrees(leftPod.getEstimatedAngleRadians()));
            telemetry.addData("Right angle (deg)", Math.toDegrees(rightPod.getEstimatedAngleRadians()));
            telemetry.update();
        }

        leftPod.stop();
        rightPod.stop();
    }

    private double shapeAxis(double value) {
        return DifferentialSwerveMath.shapeInput(
                DifferentialSwerveMath.applyDeadband(value, DifferentialSwerveConfig.INPUT_DEADBAND)
        );
    }
}
