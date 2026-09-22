package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.function.LongSupplier;

/** OpMode-scoped owner of differential-swerve hardware, snapshots, and output authorization. */
public final class DifferentialSwerveRuntime
        implements DifferentialPod.OutputGate, SafePedroFollower.CycleGuard, AutoCloseable {
    public enum State { NEW, ALIGNING, READY, ARMED, FAULT_LATCHED, STOPPED }

    public static final class Cycle {
        public final double seconds;
        public final boolean recovered;
        private Cycle(double seconds, boolean recovered) {
            this.seconds = seconds;
            this.recovered = recovered;
        }
    }

    private static final int TELEMETRY_INTERVAL_MS = 100;

    interface Host {
        DifferentialPod[] initialize(DifferentialPod.OutputGate gate, double maxDrive, double maxMotorCommand);
        String refreshHubs();
        void stopAcquiredMotors();
        void applyMotorPidfIfChanged();
        boolean stopRequested();
        boolean started();
        void pause();
        void idle();
        void showAlignment(DifferentialPod left, DifferentialPod right);
        void showHealth(int recovered, String failure);
        void log(String message);
        void close();
    }

    private final Host host;
    private final boolean autonomous;
    private final double maxDrive;
    private final LongSupplier clock;
    private DifferentialPod leftPod;
    private DifferentialPod rightPod;
    private HubSnapshotReader snapshots;
    private State state = State.NEW;
    private String fault = "none";
    private boolean faultLatched;
    private long lastSampleNanos;
    private long authorizedSnapshotNanos;
    private boolean driveRequested;
    private boolean maintenanceFloatAllowed;
    private boolean closed;
    public DifferentialSwerveRuntime(LinearOpMode opMode, boolean autonomous) {
        this(new DifferentialSwerveHardware(opMode), autonomous, System::nanoTime);
    }

    DifferentialSwerveRuntime(Host host, boolean autonomous, LongSupplier clock) {
        this.host = host;
        this.autonomous = autonomous;
        this.maxDrive = autonomous ? PedroDriveConfig.AUTONOMOUS_MAX_DRIVE : PedroDriveConfig.TELEOP_MAX_DRIVE;
        this.clock = clock;
    }

    /** Initializes, aligns, and leaves the drivetrain zeroed in READY. */
    public boolean initializeAndAlign() {
        if (closed || state != State.NEW) throw new IllegalStateException("Runtime already initialized");
        try {
            PedroDriveConfig.validate();
            DifferentialPod[] pods = host.initialize(this, maxDrive,
                    autonomous ? PedroDriveConfig.MAX_COMBINED_MOTOR_COMMAND : 1.0);
            leftPod = pods[0];
            rightPod = pods[1];
            snapshots = new HubSnapshotReader(host::refreshHubs, this::safeZeroAll,
                    host::pause, host::stopRequested, clock);
            state = State.ALIGNING;
            if (!alignPods()) {
                state = State.STOPPED;
                safeZeroAll();
                return false;
            }
            safeZeroAll();
            state = State.READY;
            return true;
        } catch (RuntimeException exception) {
            latchFault("Initialization failed: " + exception.getMessage());
            close();
            throw exception;
        }
    }

    private boolean alignPods() {
        leftPod.startAlignment();
        rightPod.startAlignment();
        long previous = clock.getAsLong();
        long nextTelemetry = 0L;
        while (!host.stopRequested()) {
            long sample = readSnapshot();
            if (sample == 0L || host.stopRequested()) return false;
            double seconds = validateSampleSeconds(previous, sample);
            previous = sample;
            authorizedSnapshotNanos = sample;
            leftPod.prepareAlignment(seconds);
            rightPod.prepareAlignment(seconds);
            if (leftPod.alignmentFailed() || rightPod.alignmentFailed()) {
                throw new IllegalStateException("Alignment: left " + leftPod.alignmentStatus()
                        + "; right " + rightPod.alignmentStatus());
            }
            validateSnapshotAge(sample, clock.getAsLong());
            leftPod.commitPrepared();
            rightPod.commitPrepared();
            if (leftPod.alignmentComplete() && rightPod.alignmentComplete()) return true;
            if (sample >= nextTelemetry) {
                nextTelemetry = sample + TELEMETRY_INTERVAL_MS * 1_000_000L;
                showHubHealth();
                host.showAlignment(leftPod, rightPod);
            }
            host.idle();
        }
        return false;
    }

    public void arm() {
        try { armInternal(); }
        catch (RuntimeException exception) {
            latchFault("Arming failed: " + exception.getMessage());
            throw exception;
        }
    }

    private void armInternal() {
        if (state != State.READY) throw new IllegalStateException("Runtime is not READY: " + state);
        long sample = readSnapshot();
        if (sample == 0L || host.stopRequested()) throw new IllegalStateException("Stop requested");
        leftPod.seedAtStart();
        rightPod.seedAtStart();
        lastSampleNanos = sample;
        authorizedSnapshotNanos = 0L;
        leftPod.setToBreak();
        rightPod.setToBreak();
        state = State.ARMED;
    }

    public Cycle beginHardwareCycle() {
        try { return beginHardwareCycleInternal(); }
        catch (RuntimeException exception) {
            latchFault("Hardware cycle failed: " + exception.getMessage());
            throw exception;
        }
    }

    private Cycle beginHardwareCycleInternal() {
        authorizedSnapshotNanos = 0L;
        driveRequested = false;
        if (state != State.ARMED) throw new HubSnapshotReader.FeedbackFault("Drive is not armed: " + state);
        int recoveredBefore = snapshots.getRecoveredReads();
        long sample = readSnapshot();
        if (sample == 0L || host.stopRequested()) {
            throw new HubSnapshotReader.FeedbackFault("Stop requested");
        }
        double seconds = validateSampleSeconds(lastSampleNanos, sample);
        lastSampleNanos = sample;
        leftPod.acceptSample(seconds);
        rightPod.acceptSample(seconds);
        host.applyMotorPidfIfChanged();
        boolean recovered = snapshots.getRecoveredReads() != recoveredBefore;
        if (recovered) {
            leftPod.verifyRecoveredAngle();
            rightPod.verifyRecoveredAngle();
            leftPod.prepareZero();
            rightPod.prepareZero();
            safeZeroAll();
            if (autonomous) {
                String message = "Autonomous canceled after recovered hub communication";
                latchFault(message);
                throw new HubSnapshotReader.FeedbackFault(message);
            }
        }
        authorizedSnapshotNanos = sample;
        driveRequested = false;
        return new Cycle(seconds, recovered);
    }

    @Override
    public double beginCycle() {
        return beginHardwareCycle().seconds;
    }

    @Override
    public void finishCycle() {
        try {
            if (state == State.ARMED && !driveRequested) safeZeroAll();
        } catch (RuntimeException exception) {
            latchFault("Cycle finalization failed: " + exception.getMessage());
            throw exception;
        } finally {
            authorizedSnapshotNanos = 0L;
            driveRequested = false;
        }
    }

    void markDriveRequested() { driveRequested = true; }

    @Override
    public void requireOutputAllowed() {
        if (state != State.ARMED || authorizedSnapshotNanos == 0L || host.stopRequested()) {
            throw new HubSnapshotReader.FeedbackFault("Nonzero output is not authorized in " + state);
        }
        validateSnapshotAge(authorizedSnapshotNanos, clock.getAsLong());
    }

    @Override public void requireAlignmentOutputAllowed() {
        if (state != State.ALIGNING || authorizedSnapshotNanos == 0L || host.stopRequested()) {
            throw new HubSnapshotReader.FeedbackFault("Alignment output is not authorized in " + state);
        }
        validateSnapshotAge(authorizedSnapshotNanos, clock.getAsLong());
    }

    public void safeZeroAll() {
        authorizedSnapshotNanos = 0L;
        driveRequested = false;
        Cleanup.runAll(
                () -> { if (leftPod != null) leftPod.safeZero(); },
                () -> { if (rightPod != null) rightPod.safeZero(); },
                () -> { if (leftPod == null || rightPod == null) host.stopAcquiredMotors(); });
    }

    @Override
    public void latchFault(String message) {
        if (!hasFault()) fault = message == null ? "unspecified fault" : message;
        faultLatched = true;
        state = State.FAULT_LATCHED;
        authorizedSnapshotNanos = 0L;
        try { safeZeroAll(); }
        catch (RuntimeException stopFailure) { host.log("Fault stop failed: " + stopFailure); }
        host.log(fault);
    }

    public void abort(String message) { latchFault(message); }
    @Override public boolean hasFault() { return faultLatched; }
    public State state() { return state; }
    public String fault() { return fault; }
    public boolean isArmed() { return state == State.ARMED; }
    public DifferentialPod leftPod() { return leftPod; }
    public DifferentialPod rightPod() { return rightPod; }
    public int recoveredReads() { return snapshots == null ? 0 : snapshots.getRecoveredReads(); }

    public void showHubHealth() {
        host.showHealth(recoveredReads(), snapshots == null ? "none" : snapshots.getLastFailure());
    }

    public void setMaintenanceFloat() {
        if (closed || (state != State.READY && state != State.STOPPED)) {
            safeZeroAll();
            throw new IllegalStateException("Maintenance FLOAT requires a disarmed runtime");
        }
        maintenanceFloatAllowed = true;
        try {
            safeZeroAll();
            leftPod.setToFloat();
            rightPod.setToFloat();
        } catch (RuntimeException exception) {
            latchFault("Maintenance FLOAT failed: " + exception.getMessage());
            try { Cleanup.runAll(leftPod::setToBreak, rightPod::setToBreak); }
            catch (RuntimeException brakeFailure) { exception.addSuppressed(brakeFailure); }
            throw exception;
        } finally {
            maintenanceFloatAllowed = false;
        }
    }

    @Override public boolean allowMaintenanceFloat() { return maintenanceFloatAllowed; }
    @Override public double snapshotAgeMillis() {
        long sample = authorizedSnapshotNanos == 0L ? lastSampleNanos : authorizedSnapshotNanos;
        return sample == 0L ? Double.POSITIVE_INFINITY : (clock.getAsLong() - sample) * 1e-6;
    }
    @Override public String faultDescription() { return fault; }

    private long readSnapshot() {
        long started = clock.getAsLong();
        long sampled = snapshots.read();
        if (sampled == 0L || host.stopRequested()) return 0L;
        validateSnapshotAge(started, sampled);
        return sampled;
    }

    static double validateSampleSeconds(long previous, long now) {
        double seconds = (now - previous) * 1e-9;
        if (!Double.isFinite(seconds) || seconds <= 0.0
                || seconds > PedroDriveConfig.MAX_FEEDBACK_AGE_SECONDS) {
            throw new HubSnapshotReader.FeedbackFault("Feedback/control delay exceeded 250 ms");
        }
        return Math.max(1e-6, seconds);
    }

    static void validateSnapshotAge(long sampled, long now) {
        double age = (now - sampled) * 1e-9;
        if (!Double.isFinite(age) || age < 0.0 || age > PedroDriveConfig.MAX_FEEDBACK_AGE_SECONDS) {
            throw new HubSnapshotReader.FeedbackFault("Feedback/control delay exceeded 250 ms");
        }
    }

    @Override
    public void close() {
        if (closed) return;
        closed = true;
        try {
            Cleanup.runAll(this::safeZeroAll,
                    () -> { if (leftPod != null) leftPod.setToBreak(); },
                    () -> { if (rightPod != null) rightPod.setToBreak(); },
                    host::close);
        } catch (RuntimeException failure) {
            host.log("Cleanup failure: " + failure);
            for (Throwable secondary : failure.getSuppressed()) host.log("Additional cleanup failure: " + secondary);
        } finally {
            state = State.STOPPED;
        }
    }
}
