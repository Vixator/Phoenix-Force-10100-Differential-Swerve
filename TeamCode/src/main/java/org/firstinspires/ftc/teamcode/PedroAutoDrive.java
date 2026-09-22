package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Convenience owner for one autonomous drivetrain/localizer/follower lifetime. */
public final class PedroAutoDrive implements AutoCloseable {
    private final LinearOpMode opMode;
    private DifferentialSwerveRuntime runtime;
    private DifferentialSwerveDrivetrain drivetrain;
    private GuardedPinpointLocalizer localizer;
    private SafePedroFollower follower;
    private boolean initialized;
    private boolean initializationAttempted;
    private boolean closed;

    public PedroAutoDrive(LinearOpMode opMode) { this.opMode = opMode; }

    public boolean initialize() {
        if (initializationAttempted || closed) throw new IllegalStateException("PedroAutoDrive cannot be reused");
        initializationAttempted = true;
        runtime = new DifferentialSwerveRuntime(opMode, true);
        try {
            if (!runtime.initializeAndAlign()) { close(); return false; }
            drivetrain = new DifferentialSwerveDrivetrain(runtime, true);
            // The published delegate's unavoidable constructor wait happens while zeroed/disarmed.
            localizer = new GuardedPinpointLocalizer(opMode.hardwareMap, runtime::isArmed);
            runtime.safeZeroAll();
            localizer.calibrateStationary(opMode, true);
            if (opMode.isStopRequested()) { close(); return false; }
            follower = new SafePedroFollower(localizer, drivetrain, new ManualOnlyAlgorithm(), runtime);
            follower.holdEnd.set(false);
            initialized = true;
            opMode.telemetry.addLine("Pedro drivetrain aligned; Pinpoint READY. Press Start when commissioning gates permit.");
            opMode.telemetry.update();
            return true;
        } catch (RuntimeException exception) {
            runtime.latchFault("Pedro initialization failed: " + exception.getMessage());
            opMode.telemetry.addLine("PEDRO INITIALIZATION FAILED");
            opMode.telemetry.addData("Fault", exception.getMessage());
            opMode.telemetry.update();
            close();
            return false;
        }
    }

    public void arm(Pose startPose) {
        if (!initialized) throw new IllegalStateException("PedroAutoDrive is not initialized");
        try {
            PinpointSettings.requirePoweredVerified();
            localizer.requireReady();
            runtime.arm();
            follower.setPose(startPose);
        } catch (RuntimeException exception) {
            runtime.latchFault("Autonomous arming failed: " + exception.getMessage());
            throw exception;
        }
    }

    public void enableForesight() {
        if (!initialized) throw new IllegalStateException("PedroAutoDrive is not initialized");
        if (runtime.isArmed()) throw new IllegalStateException("Configure Foresight before arming");
        follower.setAlgorithm(new Foresight(PedroFollowerConfig.create()));
    }

    public Follower follower() {
        if (follower == null) throw new IllegalStateException("PedroAutoDrive is not initialized");
        return follower;
    }

    public boolean hasFault() { return runtime != null && runtime.hasFault(); }
    public String fault() { return runtime == null ? "not initialized" : runtime.fault(); }

    public void abort(String message) {
        if (follower != null) {
            try { follower.stop(); } catch (RuntimeException ignored) { }
        }
        if (runtime != null) runtime.abort(message);
    }

    @Override
    public void close() {
        if (closed) return;
        closed = true;
        initialized = false;
        if (follower != null) {
            try { follower.stop(); } catch (RuntimeException ignored) { }
        }
        if (runtime != null) runtime.close();
    }
}
