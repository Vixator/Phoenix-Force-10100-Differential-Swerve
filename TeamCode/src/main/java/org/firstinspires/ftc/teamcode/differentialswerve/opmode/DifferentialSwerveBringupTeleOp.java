package org.firstinspires.ftc.teamcode.differentialswerve.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.differentialswerve.DifferentialSwerveConfig;
import org.firstinspires.ftc.teamcode.differentialswerve.DifferentialSwerveDrive;
import org.firstinspires.ftc.teamcode.differentialswerve.DifferentialSwerveMath;
import org.firstinspires.ftc.teamcode.differentialswerve.DifferentialSwervePod;

@TeleOp(name = "Differential Swerve Bringup", group = "Differential Swerve")
public class DifferentialSwerveBringupTeleOp extends LinearOpMode {

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
        DifferentialSwerveDrive drive = new DifferentialSwerveDrive(
                leftPod,
                rightPod,
                DifferentialSwerveConfig.TRACK_WIDTH_METERS
        );

        while (!isStarted() && !isStopRequested()) {
            drive.zeroPodsForward();
            telemetry.addLine("Align both pods straight forward before pressing start.");
            telemetry.addLine("This code assumes you can repeat that alignment each boot.");
            telemetry.addData("Wheel free speed (rpm)", DifferentialSwerveConfig.WHEEL_FREE_SPEED_RPM);
            telemetry.update();
        }

        waitForStart();
        drive.zeroPodsForward();

        while (opModeIsActive()) {
            drive.update();

            double slowModeScale = gamepad1.left_bumper ? 0.35 : 1.0;
            double forward = -shapeAxis(gamepad1.left_stick_y) * slowModeScale;
            double strafeLeft = shapeAxis(gamepad1.left_stick_x) * slowModeScale;
            double turn = shapeAxis(gamepad1.right_stick_x)
                    * DifferentialSwerveConfig.TURN_INPUT_SCALE
                    * slowModeScale;

            if (gamepad1.dpad_up) {
                drive.zeroPodsForward();
            }

            drive.driveRobotCentric(forward, strafeLeft, turn);

            telemetry.addLine("Controls: left stick = translate, right stick X = turn");
            telemetry.addLine("Left bumper = precision mode, D-pad up = re-zero forward");
            telemetry.addData("Left angle (deg)", Math.toDegrees(drive.getLeftPod().getEstimatedAngleRadians()));
            telemetry.addData("Right angle (deg)", Math.toDegrees(drive.getRightPod().getEstimatedAngleRadians()));
            telemetry.addData("Left encoders", "%d / %d",
                    drive.getLeftPod().getMotorAPosition(),
                    drive.getLeftPod().getMotorBPosition());
            telemetry.addData("Right encoders", "%d / %d",
                    drive.getRightPod().getMotorAPosition(),
                    drive.getRightPod().getMotorBPosition());
            telemetry.addData("Wheel free speed (rpm)", "%.1f", DifferentialSwerveConfig.WHEEL_FREE_SPEED_RPM);
            telemetry.update();
        }

        drive.stop();
    }

    private double shapeAxis(double value) {
        return DifferentialSwerveMath.shapeInput(
                DifferentialSwerveMath.applyDeadband(value, DifferentialSwerveConfig.INPUT_DEADBAND)
        );
    }
}
