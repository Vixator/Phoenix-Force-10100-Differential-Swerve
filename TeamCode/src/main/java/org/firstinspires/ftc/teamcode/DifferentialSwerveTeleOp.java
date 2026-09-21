package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

@TeleOp(name = "Differential Swerve TeleOp", group = "Differential Swerve")
public class DifferentialSwerveTeleOp extends LinearOpMode {
    private static final double MAX_MOTOR_TICKS_PER_SECOND = HardwareConstants.MAX_MOTOR_TICKS_PER_SECOND;
    private static final int TELEMETRY_INTERVAL_MS = 100;
    private static final double MAX_LOOP_SECONDS = 0.25;
    private static final double MAX_DRIVE_POWER = 1.0;

    // Indexed by the documented Control Hub ports: left pod 0/1, right pod 2/3.
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private DcMotor leftQuadrature;
    private DcMotor rightQuadrature;
    private AnalogInput leftAbsolute;
    private AnalogInput rightAbsolute;
    private final SwervePodEncoder leftEncoder = new SwervePodEncoder(SwervePodEncoder.LEFT_QUADRATURE_SIGN);
    private final SwervePodEncoder rightEncoder = new SwervePodEncoder(SwervePodEncoder.RIGHT_QUADRATURE_SIGN);
    private final DifferentialSwervePodController leftController = new DifferentialSwervePodController();
    private final DifferentialSwervePodController rightController = new DifferentialSwervePodController();
    private HubSnapshotReader snapshots;
    private double appliedMotorP = Double.NaN;
    private double appliedMotorI = Double.NaN;
    private double appliedMotorD = Double.NaN;
    private double appliedMotorF = Double.NaN;

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        LynxModule.BulkCachingMode[] previousModes = new LynxModule.BulkCachingMode[hubs.size()];
        for (int i = 0; i < hubs.size(); i++) previousModes[i] = hubs.get(i).getBulkCachingMode();
        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            initializeHardware();
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
            snapshots = new HubSnapshotReader(() -> refreshHubs(hubs), this::stopAllMotors,
                    () -> sleep(HubSnapshotReader.RETRY_DELAY_MS), this::isStopRequested, System::nanoTime);
            if (!SwervePodEncoder.calibrationReady()) {
                throw new IllegalStateException("Pod encoder calibration is not verified");
            }
            telemetry.addLine("ROBOT-CENTRIC: left stick = translation, right stick X = turn.");
            telemetry.addLine("INIT aligns both pods to forward. Keep clear while they move.");
            telemetry.update();
            if (!alignPodsToForward()) return;
            telemetry.addLine("READY — pods aligned. Press Start to drive.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            // Preserve the actual residual angle instead of pretending alignment is exact.
            long lastSample = readSnapshot();
            if (isStopRequested()) return;
            seedPod(leftEncoder, leftAbsolute, leftQuadrature,
                    SwervePodEncoder.LEFT_FORWARD_DEGREES, SwervePodEncoder.LEFT_ANALOG_SIGN);
            seedPod(rightEncoder, rightAbsolute, rightQuadrature,
                    SwervePodEncoder.RIGHT_FORWARD_DEGREES, SwervePodEncoder.RIGHT_ANALOG_SIGN);

            SwerveDriverInput input = new SwerveDriverInput();
            DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
            long nextTelemetry = 0;
            int recoveredReads = snapshots.getRecoveredReads();
            boolean waitingForNeutral = false;
            while (opModeIsActive()) {
                long sample = readSnapshot();
                if (isStopRequested()) return;
                double seconds = sampleSeconds(lastSample, sample);
                lastSample = sample;
                leftEncoder.update(rawCount(leftQuadrature), seconds);
                rightEncoder.update(rawCount(rightQuadrature), seconds);
                applyMotorPidfIfChanged();
                input.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                if (snapshots.getRecoveredReads() != recoveredReads) {
                    recoveredReads = snapshots.getRecoveredReads();
                    waitingForNeutral = true;
                    leftController.stopOutput();
                    rightController.stopOutput();
                    // A reset hub may return real snapshots with reset quadrature counts.
                    // Compare without reseeding; inconsistent feedback requires reinitialization.
                    verifyRecoveredAngle(leftEncoder, leftAbsolute,
                            SwervePodEncoder.LEFT_FORWARD_DEGREES, SwervePodEncoder.LEFT_ANALOG_SIGN);
                    verifyRecoveredAngle(rightEncoder, rightAbsolute,
                            SwervePodEncoder.RIGHT_FORWARD_DEGREES, SwervePodEncoder.RIGHT_ANALOG_SIGN);
                }
                if (waitingForNeutral) {
                    waitingForNeutral = input.getForward() != 0.0 || input.getStrafe() != 0.0 || input.getTurn() != 0.0;
                    // Skip outputs even on the neutral transition; resume on the next fresh read.
                    if (sample >= nextTelemetry) {
                        nextTelemetry = sample + TELEMETRY_INTERVAL_MS * 1_000_000L;
                        telemetry.addLine("Hub feedback recovered. Center both sticks to resume.");
                        showHubHealth();
                        telemetry.update();
                    }
                    continue;
                }
                kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
                updatePod(leftController, leftEncoder, kinematics.getLeftTargetAngle(), kinematics.getLeftSpeed(), seconds);
                updatePod(rightController, rightEncoder, kinematics.getRightTargetAngle(), kinematics.getRightSpeed(), seconds);

                if (isStopRequested()) return;
                sampleSeconds(sample, System.nanoTime()); // Reject commands based on excessively old feedback.
                commandPod(0, leftController.getLeftMotorCommand(), leftController.getRightMotorCommand());
                commandRightPod(rightController.getLeftMotorCommand(), rightController.getRightMotorCommand());
                if (sample >= nextTelemetry) {
                    nextTelemetry = sample + TELEMETRY_INTERVAL_MS * 1_000_000L;
                    telemetry.addLine("ROBOT-CENTRIC");
                    showPod("Left", leftEncoder, leftController, false, kinematics.getLeftSpeed());
                    showPod("Right", rightEncoder, rightController, true, kinematics.getRightSpeed());
                    telemetry.addData("Forward / right / CW rad/s", "%.2f / %.2f / %.2f",
                            input.getForward(), input.getStrafe(), input.getTurn());
                    telemetry.addData("Module wheel speed (m/s)", "left %+.3f / right %+.3f",
                            moduleSpeedMetersPerSecond(0), moduleSpeedMetersPerSecond(2));
                    telemetry.addData("Loop (ms)", "%.1f", seconds * 1000.0);
                    showHubHealth();
                    telemetry.update();
                }
            }
        } catch (HubSnapshotReader.FeedbackFault fault) {
            holdFeedbackFault(fault.getMessage());
        } finally {
            // Attempt every stop/restore even if a disconnected device rejects cleanup.
            try {
                stopAllMotors();
            } catch (RuntimeException exception) {
                RobotLog.ee("DifferentialSwerve", "Final motor stop failed: " + exception);
            }
            for (int i = 0; i < hubs.size(); i++) {
                try {
                    hubs.get(i).setBulkCachingMode(previousModes[i]);
                } catch (RuntimeException exception) {
                    RobotLog.ee("DifferentialSwerve", "Cache mode restore failed: " + exception);
                }
            }
        }
    }

    private void initializeHardware() {
        String[] names = {HardwareConstants.MOTOR_LEFT_POD_LEFT, HardwareConstants.MOTOR_LEFT_POD_RIGHT,
                HardwareConstants.MOTOR_RIGHT_POD_LEFT, HardwareConstants.MOTOR_RIGHT_POD_RIGHT};
        for (int i = 0; i < motors.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
            motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setVelocity(0.0);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (motors[i].getPortNumber() != i || motors[i].getController() != motors[0].getController()) {
                throw new IllegalArgumentException("Configure motor0..motor3 on Control Hub ports 0..3");
            }
        }
        leftQuadrature = hardwareMap.get(DcMotor.class, HardwareConstants.ENCODER_LEFT);
        rightQuadrature = hardwareMap.get(DcMotor.class, HardwareConstants.ENCODER_RIGHT);
        if (leftQuadrature.getPortNumber() != 0 || rightQuadrature.getPortNumber() != 1
                || leftQuadrature.getController() != rightQuadrature.getController()
                || leftQuadrature.getController() == motors[0].getController()) {
            throw new IllegalArgumentException("Configure encoderleft/encoderright on separate Expansion Hub ports 0/1");
        }
        leftAbsolute = hardwareMap.get(AnalogInput.class, SwervePodEncoder.LEFT_ANALOG_NAME);
        rightAbsolute = hardwareMap.get(AnalogInput.class, SwervePodEncoder.RIGHT_ANALOG_NAME);
        applyMotorPidfIfChanged();
    }

    private boolean alignPodsToForward() {
        PodAlignmentController left = new PodAlignmentController(
                SwervePodEncoder.LEFT_FORWARD_DEGREES, SwervePodEncoder.LEFT_ANALOG_SIGN);
        PodAlignmentController right = new PodAlignmentController(
                SwervePodEncoder.RIGHT_FORWARD_DEGREES, SwervePodEncoder.RIGHT_ANALOG_SIGN);
        left.start();
        right.start();
        long lastSample = System.nanoTime();
        long nextTelemetry = 0;
        // opModeIsActive() is false in INIT. Stop is the only lifecycle cancellation
        // here, so an early Start cannot skip unfinished alignment.
        while (!isStopRequested()) {
            long sample = readSnapshot();
            if (isStopRequested()) return false;
            double seconds = sampleSeconds(lastSample, sample);
            lastSample = sample;
            double leftVolts = leftAbsolute.getVoltage();
            double rightVolts = rightAbsolute.getVoltage();
            // Continue checking a completed pod while its partner is still aligning.
            recheckAlignment(left, leftVolts, SwervePodEncoder.LEFT_ANALOG_SIGN);
            recheckAlignment(right, rightVolts, SwervePodEncoder.RIGHT_ANALOG_SIGN);
            left.step(leftVolts, seconds);
            right.step(rightVolts, seconds);
            if (left.isFailed() || right.isFailed()) {
                throw new IllegalStateException("Alignment: " + SwervePodEncoder.LEFT_ANALOG_NAME + " "
                        + left.getStatus() + " (" + leftVolts + " V); " + SwervePodEncoder.RIGHT_ANALOG_NAME
                        + " " + right.getStatus() + " (" + rightVolts + " V)");
            }
            if (isStopRequested()) return false;
            sampleSeconds(sample, System.nanoTime());
            commandPod(0, left.getCommand(), -left.getCommand());
            commandRightPod(right.getCommand(), -right.getCommand());
            if (left.isComplete() && right.isComplete()) return true;
            if (sample >= nextTelemetry) {
                nextTelemetry = sample + TELEMETRY_INTERVAL_MS * 1_000_000L;
                telemetry.addLine(isStarted() ? "FINISHING ALIGNMENT — drive waiting" : "INIT — ALIGNING PODS TO FORWARD");
                telemetry.addData("Left", "%s | %.4f V | error %.1f deg", left.getStatus(), leftVolts, left.getErrorDegrees());
                telemetry.addData("Right", "%s | %.4f V | error %.1f deg", right.getStatus(), rightVolts, right.getErrorDegrees());
                showHubHealth();
                telemetry.update();
            }
            idle();
        }
        return false;
    }

    private static void recheckAlignment(PodAlignmentController alignment, double volts, int sign) {
        if (!SwervePodEncoder.validVoltage(volts)) {
            alignment.abort(SwervePodEncoder.voltageFault(volts));
        } else if (alignment.isComplete() && Math.abs(SwervePodEncoder.analogErrorDegrees(
                volts, alignment.getTargetDegrees(), sign)) > PodAlignmentController.TOLERANCE_DEGREES) {
            alignment.abort("moved away from forward after settling");
        }
    }

    /** Explicit bulk reads refresh MANUAL caches; getters below reuse these snapshots. */
    private long readSnapshot() {
        long started = System.nanoTime();
        long sampled = snapshots.read();
        if (isStopRequested()) return 0;
        sampleSeconds(started, sampled);
        return sampled;
    }

    private String refreshHubs(List<LynxModule> hubs) {
        for (LynxModule hub : hubs) {
            if (isStopRequested()) return "Stop requested";
            String failure = null;
            try {
                if (hub.getBulkData().isFake()) failure = "SDK returned fake bulk data";
            } catch (RuntimeException exception) {
                failure = exception.getClass().getSimpleName() + ": " + exception.getMessage();
            }
            if (failure != null) {
                String detail = hardwareMap.getNamesOf(hub) + " | " + hub.getConnectionInfo()
                        + " | " + (hub.isParent() ? "parent" : "downstream") + " | " + failure;
                RobotLog.ww("DifferentialSwerve", detail);
                return detail;
            }
        }
        return null;
    }

    private void showHubHealth() {
        telemetry.addData("Recovered hub reads", snapshots.getRecoveredReads());
        if (snapshots.getRecoveredReads() > 0) telemetry.addData("Last hub read failure", snapshots.getLastFailure());
    }

    private void holdFeedbackFault(String message) {
        RobotLog.ee("DifferentialSwerve", message);
        // Remain latched until Stop, including if Start is pressed during an INIT fault.
        while (!isStopRequested()) {
            String stopFailure = "";
            try {
                stopAllMotors();
            } catch (RuntimeException exception) {
                stopFailure = exception.toString();
            }
            telemetry.addLine("DRIVE STOPPED — hub feedback fault. Stop and reinitialize.");
            telemetry.addData("Fault", message);
            if (!stopFailure.isEmpty()) telemetry.addData("Motor stop delivery failed", stopFailure);
            telemetry.update();
            sleep(TELEMETRY_INTERVAL_MS);
        }
    }

    private static void verifyRecoveredAngle(SwervePodEncoder encoder, AnalogInput absolute,
                                             double forward, int sign) {
        double volts = absolute.getVoltage();
        if (!SwervePodEncoder.validVoltage(volts)) {
            throw new HubSnapshotReader.FeedbackFault(SwervePodEncoder.voltageFault(volts));
        }
        double difference = SwervePodEncoder.wrapRadians(encoder.getAngleRadians()
                - SwervePodEncoder.absoluteRadians(volts, forward, sign));
        if (Math.abs(difference) > Math.toRadians(10.0)) {
            throw new HubSnapshotReader.FeedbackFault("Pod feedback disagrees after hub recovery: "
                    + absolute.getConnectionInfo() + " | difference " + Math.toDegrees(difference) + " deg");
        }
    }

    private static double sampleSeconds(long previous, long now) {
        double seconds = (now - previous) * 1e-9;
        if (seconds > MAX_LOOP_SECONDS) throw new HubSnapshotReader.FeedbackFault("Feedback/control delay exceeded 250 ms");
        return Math.max(1e-6, seconds);
    }

    private static int rawCount(DcMotor encoder) {
        return encoder.getController().getMotorCurrentPosition(encoder.getPortNumber());
    }

    private static void seedPod(SwervePodEncoder encoder, AnalogInput absolute, DcMotor quadrature,
                                double forwardDegrees, int analogSign) {
        double volts = absolute.getVoltage();
        if (!SwervePodEncoder.validVoltage(volts)) {
            throw new IllegalStateException(absolute.getConnectionInfo() + ": " + SwervePodEncoder.voltageFault(volts));
        }
        double angle = SwervePodEncoder.absoluteRadians(volts, forwardDegrees, analogSign);
        if (Math.abs(Math.toDegrees(angle)) > PodAlignmentController.TOLERANCE_DEGREES) {
            throw new IllegalStateException("Pod moved after INIT alignment: " + absolute.getConnectionInfo()
                    + " | " + volts + " V | angle " + Math.toDegrees(angle) + " deg. Reinitialize to realign.");
        }
        encoder.seed(angle, rawCount(quadrature));
    }

    private static void updatePod(DifferentialSwervePodController controller, SwervePodEncoder encoder,
                                   double target, double speed, double seconds) {
        SwerveTuning.validate();
        controller.update(encoder.getAngleRadians(), encoder.getRateRadiansPerSecond(), target, speed, seconds,
                SwerveTuning.STEERING_P, SwerveTuning.STEERING_D,
                MAX_DRIVE_POWER, SwerveTuning.STEERING_MAX_COMMAND,
                SwerveTuning.STEERING_SLEW_RATE);
    }

    private void applyMotorPidfIfChanged() {
        SwerveTuning.validate();
        double p = SwerveTuning.MOTOR_VELOCITY_P;
        double i = SwerveTuning.MOTOR_VELOCITY_I;
        double d = SwerveTuning.MOTOR_VELOCITY_D;
        double f = SwerveTuning.MOTOR_VELOCITY_F;
        if (p == appliedMotorP && i == appliedMotorI && d == appliedMotorD && f == appliedMotorF) return;
        for (DcMotorEx motor : motors) {
            if (motor != null) motor.setVelocityPIDFCoefficients(p, i, d, f);
        }
        appliedMotorP = p;
        appliedMotorI = i;
        appliedMotorD = d;
        appliedMotorF = f;
    }

    private void commandPod(int firstMotor, double left, double right) {
        motors[firstMotor].setVelocity(left * MAX_MOTOR_TICKS_PER_SECOND);
        motors[firstMotor + 1].setVelocity(right * MAX_MOTOR_TICKS_PER_SECOND);
    }

    private void commandRightPod(double left, double right) {
        // Measured right-pod polarity reverses common wheel drive but preserves differential steering.
        commandPod(2, -right, -left);
    }

    private void showPod(String name, SwervePodEncoder encoder,
                         DifferentialSwervePodController controller, boolean rightPod, double requestedSpeed) {
        telemetry.addData(name + " angle / target / error (deg)", "%.1f / %.1f / %.1f",
                Math.toDegrees(encoder.getAngleRadians()), Math.toDegrees(controller.getOptimizedTargetAngle()),
                Math.toDegrees(controller.getAngleError()));
        double leftCommand = controller.getLeftMotorCommand();
        double rightCommand = controller.getRightMotorCommand();
        if (rightPod) {
            double logicalLeft = leftCommand;
            leftCommand = -rightCommand;
            rightCommand = -logicalLeft;
        }
        telemetry.addData(name + " motor targets (ticks/s)", "%.0f / %.0f",
                leftCommand * MAX_MOTOR_TICKS_PER_SECOND, rightCommand * MAX_MOTOR_TICKS_PER_SECOND);
        // Scalar values can be graphed in Dashboard. Individual speeds distinguish a
        // stopped motor from opposite motor speeds that cancel in the wheel average.
        int firstMotor = rightPod ? 2 : 0;
        showMotor(firstMotor, leftCommand);
        showMotor(firstMotor + 1, rightCommand);
        telemetry.addData(name + " requested speed (normalized)", requestedSpeed);
        telemetry.addData(name + " drive command",
                (controller.getLeftMotorCommand() + controller.getRightMotorCommand()) / 2.0);
        telemetry.addData(name + " steer command", controller.getSteeringCommand());
        telemetry.addData(name + " angle error (deg)", Math.toDegrees(controller.getAngleError()));
        telemetry.addData(name + " quadrature count", encoder.getCount());
        telemetry.addData(name + " quadrature angle (deg)", Math.toDegrees(encoder.getAngleRadians()));
        telemetry.addData(name + " pod rate (deg/s)", Math.toDegrees(encoder.getRateRadiansPerSecond()));

        // Diagnostic only: use the cached analog sample, without reseeding the tracker
        // or altering control if this independent position measurement disagrees.
        double volts = (rightPod ? rightAbsolute : leftAbsolute).getVoltage();
        telemetry.addData(name + " absolute voltage", volts);
        double absoluteAngle = SwervePodEncoder.validVoltage(volts)
                ? SwervePodEncoder.absoluteRadians(volts,
                        rightPod ? SwervePodEncoder.RIGHT_FORWARD_DEGREES : SwervePodEncoder.LEFT_FORWARD_DEGREES,
                        rightPod ? SwervePodEncoder.RIGHT_ANALOG_SIGN : SwervePodEncoder.LEFT_ANALOG_SIGN)
                : Double.NaN;
        telemetry.addData(name + " absolute angle (deg)", Math.toDegrees(absoluteAngle));
        telemetry.addData(name + " quadrature minus absolute (deg)",
                Math.toDegrees(SwervePodEncoder.wrapRadians(encoder.getAngleRadians() - absoluteAngle)));
    }

    private void showMotor(int index, double command) {
        // Velocity and position reuse the existing bulk snapshot; do not add slow
        // per-motor current transactions to the feedback/control loop.
        telemetry.addData("motor" + index + " target (ticks/s)", command * MAX_MOTOR_TICKS_PER_SECOND);
        telemetry.addData("motor" + index + " measured (ticks/s)", motors[index].getVelocity());
        telemetry.addData("motor" + index + " count", motors[index].getCurrentPosition());
    }

    private double moduleSpeedMetersPerSecond(int firstMotor) {
        double averageMotorTicksPerSecond =
                (motors[firstMotor].getVelocity() + motors[firstMotor + 1].getVelocity()) / 2.0;
        if (firstMotor == 2) averageMotorTicksPerSecond = -averageMotorTicksPerSecond;
        return averageMotorTicksPerSecond / HardwareConstants.MOTOR_TICKS_PER_REVOLUTION
                * HardwareConstants.TOTAL_DRIVE_RATIO * HardwareConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    private void stopAllMotors() {
        RuntimeException failure = null;
        for (DcMotorEx motor : motors) {
            if (motor == null) continue; // Also covers a partial hardware initialization failure.
            try {
                motor.setVelocity(0.0);
            } catch (RuntimeException exception) {
                if (failure == null) failure = exception;
            }
        }
        if (failure != null) throw failure;
    }
}
