package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Selected-pod azimuth diagnostic; acquires no other pod motors or Pinpoint. */
@TeleOp(name = "Pedro Pod Angle Test", group = "Pedro Commissioning")
public final class PedroPodAngleTest extends LinearOpMode {
    private static final boolean TEST_RIGHT_POD = false;

    @Override public void runOpMode() {
        try (Session session = new Session()) {
            session.initialize();
            DifferentialPod pod = session.pod;
            session.aligning = true;
            pod.startAlignment();
            while (!isStopRequested() && !pod.alignmentComplete()) {
                double seconds = session.sample();
                pod.prepareAlignment(seconds);
                if (pod.alignmentFailed()) throw new IllegalStateException(pod.alignmentStatus());
                pod.commitPrepared();
                telemetry.addData("Selected pod / alignment", "%s / %s", pod.name(), pod.alignmentStatus());
                telemetry.update();
                idle();
            }
            pod.safeZero();
            session.aligning = false;
            if (isStopRequested()) return;
            telemetry.addLine("READY. D-pad up/right/left/down: 0/+45/-45/+90; A: -90; bumpers: +/-179 deg.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            session.previous = System.nanoTime();
            session.sample();
            pod.seedAtStart();
            session.armed = true;
            double targetCw = 0.0;
            while (opModeIsActive()) {
                pod.acceptSample(session.sample());
                if (gamepad1.dpad_up) targetCw = 0.0;
                if (gamepad1.dpad_right) targetCw = Math.toRadians(45.0);
                if (gamepad1.dpad_left) targetCw = Math.toRadians(-45.0);
                if (gamepad1.dpad_down) targetCw = Math.toRadians(90.0);
                if (gamepad1.a) targetCw = Math.toRadians(-90.0);
                if (gamepad1.right_bumper) targetCw = Math.toRadians(179.0);
                if (gamepad1.left_bumper) targetCw = Math.toRadians(-179.0);
                pod.move(PedroSwerveMath.wheelAngle(targetCw), 0.0, false);
                telemetry.addData("Selected pod", pod.name());
                telemetry.addData("Target / actual CW deg", "%.1f / %.1f",
                        Math.toDegrees(targetCw), Math.toDegrees(pod.getAngle()));
                telemetry.addData("Debug", pod.debug());
                telemetry.update();
                idle();
            }
        }
    }

    private final class Session implements DifferentialPod.OutputGate, AutoCloseable {
        final DcMotorEx[] motors = new DcMotorEx[2];
        final SwerveHubSession hubs = new SwerveHubSession(hardwareMap, PedroPodAngleTest.this::isStopRequested);
        final HubSnapshotReader reader = new HubSnapshotReader(hubs::refresh, this::zero,
                () -> sleep(HubSnapshotReader.RETRY_DELAY_MS), PedroPodAngleTest.this::isStopRequested, System::nanoTime);
        DifferentialPod pod;
        boolean armed, aligning;
        long sampled, previous;
        String fault = "none";

        void initialize() {
            PedroDriveConfig.validate();
            String[] names = TEST_RIGHT_POD
                    ? new String[] {HardwareConstants.MOTOR_RIGHT_POD_LEFT, HardwareConstants.MOTOR_RIGHT_POD_RIGHT}
                    : new String[] {HardwareConstants.MOTOR_LEFT_POD_LEFT, HardwareConstants.MOTOR_LEFT_POD_RIGHT};
            for (int i = 0; i < 2; i++) {
                motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
                motors[i].setVelocity(0.0);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motors[i].setVelocityPIDFCoefficients(SwerveTuning.MOTOR_VELOCITY_P,
                        SwerveTuning.MOTOR_VELOCITY_I, SwerveTuning.MOTOR_VELOCITY_D, SwerveTuning.MOTOR_VELOCITY_F);
                if (motors[i].getPortNumber() != (TEST_RIGHT_POD ? 2 : 0) + i
                        || motors[i].getController() != motors[0].getController())
                    throw new IllegalArgumentException("Selected pod motor ports/controller mismatch");
            }
            DcMotor quadrature = hardwareMap.get(DcMotor.class,
                    TEST_RIGHT_POD ? HardwareConstants.ENCODER_RIGHT : HardwareConstants.ENCODER_LEFT);
            if (quadrature.getPortNumber() != (TEST_RIGHT_POD ? 1 : 0)
                    || quadrature.getController() == motors[0].getController())
                throw new IllegalArgumentException("Selected pod quadrature port/controller mismatch");
            AnalogInput absolute = hardwareMap.get(AnalogInput.class,
                    TEST_RIGHT_POD ? HardwareConstants.ANALOG_RIGHT : HardwareConstants.ANALOG_LEFT);
            pod = DifferentialPod.forHardware(TEST_RIGHT_POD ? "rightPod" : "leftPod", TEST_RIGHT_POD,
                    motors[0], motors[1], quadrature, absolute, PedroDriveConfig.AUTONOMOUS_MAX_DRIVE,
                    PedroDriveConfig.MAX_COMBINED_MOTOR_COMMAND, this);
            hubs.initialize();
            previous = System.nanoTime();
        }
        double sample() {
            long started = System.nanoTime();
            int recoveries = reader.getRecoveredReads();
            sampled = reader.read();
            if (sampled == 0 || isStopRequested()) throw new IllegalStateException("Stop requested");
            DifferentialSwerveRuntime.validateSnapshotAge(started, sampled);
            double dt = DifferentialSwerveRuntime.validateSampleSeconds(previous, sampled);
            previous = sampled;
            if (armed && recoveries != reader.getRecoveredReads())
                throw new IllegalStateException("Diagnostic canceled after hub recovery; reinitialize");
            return dt;
        }
        void zero() {
            sampled = 0;
            Cleanup.runAll(() -> { if (motors[0] != null) motors[0].setVelocity(0); },
                    () -> { if (motors[1] != null) motors[1].setVelocity(0); });
        }
        void requirePhase(boolean allowed) {
            if (!allowed || isStopRequested() || sampled == 0 || !"none".equals(fault))
                throw new IllegalStateException("Diagnostic output is not authorized");
            DifferentialSwerveRuntime.validateSnapshotAge(sampled, System.nanoTime());
        }
        @Override public void requireOutputAllowed() { requirePhase(armed); }
        @Override public void requireAlignmentOutputAllowed() { requirePhase(aligning); }
        @Override public void latchFault(String message) { fault = message; armed = aligning = false; }
        @Override public boolean allowMaintenanceFloat() { return false; }
        @Override public double snapshotAgeMillis() { return (System.nanoTime() - sampled) * 1e-6; }
        @Override public String faultDescription() { return fault; }
        @Override public void close() {
            armed = aligning = false;
            Cleanup.runAll(this::zero,
                    () -> { if (motors[0] != null) motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); },
                    () -> { if (motors[1] != null) motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); },
                    hubs::close);
        }
    }
}
