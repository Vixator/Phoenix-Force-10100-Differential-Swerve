package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Map;

/** Established robot-centric TeleOp, now using the shared drivetrain lifecycle and safety gate. */
@TeleOp(name = "Differential Swerve TeleOp", group = "Differential Swerve")
public class DifferentialSwerveTeleOp extends LinearOpMode {
    private static final int TELEMETRY_INTERVAL_MS = 100;

    @Override
    public void runOpMode() {
        DifferentialSwerveRuntime runtime = new DifferentialSwerveRuntime(this, false);
        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
            telemetry.addLine("ROBOT-CENTRIC: left stick = translation, right stick X = turn.");
            telemetry.addLine("INIT aligns both pods to forward. Keep clear while they move.");
            telemetry.update();
            if (!runtime.initializeAndAlign()) return;
            DifferentialSwerveDrivetrain drivetrain = new DifferentialSwerveDrivetrain(runtime, false);

            telemetry.addLine("READY — pods aligned. Press Start to drive.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            runtime.arm();

            SwerveDriverInput input = new SwerveDriverInput();
            boolean waitingForNeutral = false;
            long nextTelemetry = 0L;
            while (opModeIsActive()) {
                DifferentialSwerveRuntime.Cycle cycle = runtime.beginHardwareCycle();
                try {
                    input.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                    if (cycle.recovered) waitingForNeutral = true;
                    if (waitingForNeutral) {
                        boolean nonNeutral = input.getForward() != 0.0
                                || input.getStrafe() != 0.0 || input.getTurn() != 0.0;
                        if (!nonNeutral) waitingForNeutral = false;
                    } else {
                        drivetrain.drive(new DrivePowers(
                                input.getForward(), -input.getStrafe(),
                                -input.getTurn() / SwerveDriverInput.MAX_TURN_RATE), true);
                    }
                } finally {
                    runtime.finishCycle();
                }

                long now = System.nanoTime();
                if (now >= nextTelemetry) {
                    nextTelemetry = now + TELEMETRY_INTERVAL_MS * 1_000_000L;
                    telemetry.addLine(waitingForNeutral
                            ? "Hub feedback recovered. Center both sticks to resume."
                            : "ROBOT-CENTRIC");
                    telemetry.addData("Forward / right / CW rad/s", "%.2f / %.2f / %.2f",
                            input.getForward(), input.getStrafe(), input.getTurn());
                    showPod("Left", runtime.leftPod().debug());
                    showPod("Right", runtime.rightPod().debug());
                    telemetry.addData("Loop (ms)", "%.1f", cycle.seconds * 1000.0);
                    runtime.showHubHealth();
                    telemetry.update();
                }
            }
        } catch (RuntimeException exception) {
            runtime.latchFault(exception.getMessage());
            RobotLog.ee("DifferentialSwerve", "TeleOp stopped: " + exception);
            while (!isStopRequested()) {
                telemetry.addLine("DRIVE STOPPED — feedback/control fault. Stop and reinitialize.");
                telemetry.addData("Fault", runtime.fault());
                telemetry.update();
                sleep(TELEMETRY_INTERVAL_MS);
            }
        } finally {
            runtime.close();
        }
    }

    private void showPod(String name, Map<String, Object> data) {
        int firstMotor = "Right".equals(name) ? 2 : 0;
        telemetry.addData("motor" + firstMotor + " target (ticks/s)", data.get("firstTargetTicks"));
        telemetry.addData("motor" + (firstMotor + 1) + " target (ticks/s)", data.get("secondTargetTicks"));
        telemetry.addData("motor" + firstMotor + " measured (ticks/s)", data.get("firstMeasuredTicks"));
        telemetry.addData("motor" + (firstMotor + 1) + " measured (ticks/s)", data.get("secondMeasuredTicks"));
        telemetry.addData("motor" + firstMotor + " count", data.get("firstCount"));
        telemetry.addData("motor" + (firstMotor + 1) + " count", data.get("secondCount"));
        telemetry.addData(name + " requested speed (normalized)", data.get("requestedDrive"));
        telemetry.addData(name + " drive command", data.get("driveCommand"));
        telemetry.addData(name + " steer command", data.get("steeringCommand"));
        telemetry.addData(name + " angle error (deg)", Math.toDegrees((double) data.get("errorRad")));
        telemetry.addData(name + " quadrature count", data.get("rawCount"));
        telemetry.addData(name + " quadrature angle (deg)", Math.toDegrees((double) data.get("actualEncoderRad")));
        telemetry.addData(name + " pod rate (deg/s)", Math.toDegrees((double) data.get("rateRadPerSec")));
        telemetry.addData(name + " absolute voltage", data.get("absoluteVolts"));
        telemetry.addData(name + " absolute angle (deg)", Math.toDegrees((double) data.get("absoluteAngleRad")));
        telemetry.addData(name + " quadrature minus absolute (deg)",
                Math.toDegrees((double) data.get("quadratureMinusAbsoluteRad")));
        telemetry.addData(name + " angle / target / error (deg)", "%.1f / %.1f / %.1f",
                Math.toDegrees((double) data.get("actualEncoderRad")),
                Math.toDegrees((double) data.get("optimizedTargetRad")),
                Math.toDegrees((double) data.get("errorRad")));
        telemetry.addData(name + " drive / steer", "%.3f / %.3f",
                (double) data.get("driveCommand"), (double) data.get("steeringCommand"));
        telemetry.addData(name + " quadrature / absolute V", "%s / %.4f",
                data.get("rawCount"), (double) data.get("absoluteVolts"));
        telemetry.addData(name + " rate / quad-absolute (deg)", "%.1f deg/s / %.1f deg",
                Math.toDegrees((double) data.get("rateRadPerSec")),
                Math.toDegrees((double) data.get("quadratureMinusAbsoluteRad")));
        telemetry.addData(name + " hardware targets", "%.0f / %.0f",
                (double) data.get("firstTargetTicks"), (double) data.get("secondTargetTicks"));
        telemetry.addData(name + " measured ticks/s", "%.0f / %.0f",
                (double) data.get("firstMeasuredTicks"), (double) data.get("secondMeasuredTicks"));
        telemetry.addData(name + " module speed (m/s)", "%.3f",
                (double) data.get("moduleSpeedMetersPerSecond"));
    }
}
