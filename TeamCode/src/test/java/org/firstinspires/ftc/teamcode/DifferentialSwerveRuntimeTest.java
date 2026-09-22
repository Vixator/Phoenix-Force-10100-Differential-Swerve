package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import org.junit.Test;
import static org.junit.Assert.*;

public class DifferentialSwerveRuntimeTest {
    @Test public void initAndFreshStartSeedResidualAngleWithoutMotion() {
        Host h = new Host();
        DifferentialSwerveRuntime r = h.runtime(true);
        assertTrue(r.initializeAndAlign());
        assertEquals(DifferentialSwerveRuntime.State.READY, r.state());
        h.volts[0] = volts(false, 1.0);
        h.counts[0] = 42;
        r.arm();
        assertEquals(Math.toRadians(1), r.leftPod().getAngle(), 1e-9);
        assertEquals(42, r.leftPod().quadratureCount());
        h.assertZero();
        assertThrows(HubSnapshotReader.FeedbackFault.class, r::requireOutputAllowed);
    }

    @Test public void movedPodAtStartLatchesAndStops() {
        Host h = new Host(); DifferentialSwerveRuntime r = h.runtime(true);
        r.initializeAndAlign(); h.volts[1] = volts(true, 15);
        assertThrows(IllegalStateException.class, r::arm);
        assertTrue(r.hasFault()); h.assertZero();
    }

    @Test public void earlyStartStillWaitsForAlignmentAndStopCancelsInit() {
        Host h = new Host(); h.started = true;
        DifferentialSwerveRuntime r = h.runtime(true);
        assertTrue(r.initializeAndAlign());
        assertTrue(h.now >= 1_100_000_000L);
        Host stopped = new Host(); stopped.stopped = true;
        DifferentialSwerveRuntime canceled = stopped.runtime(true);
        assertFalse(canceled.initializeAndAlign()); stopped.assertZero();
    }

    @Test public void alignmentTimeoutAndInvalidAnalogStopBothPods() {
        for (double bad : new double[] {volts(true, 20), Double.NaN}) {
            Host h = new Host(); h.volts[1] = bad;
            DifferentialSwerveRuntime r = h.runtime(true);
            assertThrows(IllegalStateException.class, r::initializeAndAlign);
            assertTrue(r.hasFault()); h.assertZero();
        }
    }

    @Test public void outputUsesRightMappingAndZeroClearsAllMotors() {
        Host h = new Host(); DifferentialSwerveRuntime r = ready(h, true);
        DifferentialSwerveDrivetrain d = new DifferentialSwerveDrivetrain(r, true);
        r.beginCycle(); d.drive(new DrivePowers(1, 0, 0), true); r.finishCycle();
        assertTrue(h.motors[0].velocity > 0);
        assertEquals(h.motors[0].velocity, -h.motors[2].velocity, 1e-9);
        r.beginCycle(); d.drive(DrivePowers.zero(), true); r.finishCycle(); h.assertZero();
    }

    @Test public void clockwiseTurnUsesPositiveHardwareCommandsOnBothMountedPods() {
        Host h = new Host(); DifferentialSwerveRuntime r = ready(h, false);
        DifferentialSwerveDrivetrain d = new DifferentialSwerveDrivetrain(r, false);
        r.beginCycle(); d.drive(new DrivePowers(0, 0, -1), true); r.finishCycle();
        for (Motor motor : h.motors) assertEquals(HardwareConstants.MAX_MOTOR_TICKS_PER_SECOND, motor.velocity, 1e-8);
        r.close();
    }

    @Test public void skippedDriveCycleStopsPreviousCommandAndRevokesAuthorization() {
        Host h = new Host(); DifferentialSwerveRuntime r = ready(h, true);
        DifferentialSwerveDrivetrain d = new DifferentialSwerveDrivetrain(r, true);
        r.beginCycle(); d.drive(new DrivePowers(.5, 0, 0), false); r.finishCycle();
        r.beginCycle(); r.finishCycle(); h.assertZero();
        assertThrows(HubSnapshotReader.FeedbackFault.class, r::requireOutputAllowed);
    }

    @Test public void staleSnapshotOrStopBetweenMotorWritesStopsEveryMotor() {
        for (boolean stop : new boolean[] {false, true}) {
            Host h = new Host(); DifferentialSwerveRuntime r = ready(h, true);
            DifferentialSwerveDrivetrain d = new DifferentialSwerveDrivetrain(r, true);
            r.beginCycle();
            h.motors[0].afterWrite = () -> { if (stop) h.stopped = true; else h.now += 251_000_000L; };
            assertThrows(RuntimeException.class, () -> d.drive(new DrivePowers(.5, 0, 0), false));
            assertTrue(r.hasFault()); h.assertZero();
            assertEquals(0, h.motors[1].nonzeroWrites);
        }
    }

    @Test public void autonomousRecoveryCancelsButTeleopReportsRecovery() {
        for (boolean auto : new boolean[] {false, true}) {
            Host h = new Host(); DifferentialSwerveRuntime r = ready(h, auto);
            h.failReads = 1;
            if (auto) {
                assertThrows(HubSnapshotReader.FeedbackFault.class, r::beginHardwareCycle);
                assertTrue(r.hasFault());
            } else {
                assertTrue(r.beginHardwareCycle().recovered);
                assertFalse(r.hasFault()); r.finishCycle();
            }
            assertEquals(1, r.recoveredReads()); h.assertZero();
        }
    }

    @Test public void persistentLossRecoveryDisagreementAndLoopOverrunLatch() {
        for (int fault = 0; fault < 3; fault++) {
            Host h = new Host(); DifferentialSwerveRuntime r = ready(h, false);
            if (fault == 0) h.failReads = 3;
            if (fault == 1) { h.failReads = 1; h.volts[0] = volts(false, 20); }
            if (fault == 2) h.now += 251_000_000L;
            assertThrows(HubSnapshotReader.FeedbackFault.class, r::beginHardwareCycle);
            assertTrue(r.hasFault()); h.assertZero();
        }
    }

    @Test public void partialMotorWriteAndStopFailureStillAttemptAllStops() {
        Host h = new Host(); DifferentialSwerveRuntime r = ready(h, true);
        DifferentialSwerveDrivetrain d = new DifferentialSwerveDrivetrain(r, true);
        r.beginCycle(); h.motors[1].failNonzero = true; h.motors[0].failZero = true;
        int before = h.motors[3].zeroWrites;
        assertThrows(RuntimeException.class, () -> d.drive(new DrivePowers(.5, 0, 0), false));
        assertTrue(r.hasFault()); assertTrue(h.motors[3].zeroWrites > before);
        r.close(); assertEquals(1, h.closes);
        assertTrue(r.hasFault()); r.close(); assertEquals(1, h.closes);
    }

    @Test public void partialInitializationStillStopsAcquiredMotorsAndClosesHost() {
        Host h = new Host(); h.failInit = true;
        DifferentialSwerveRuntime r = h.runtime(true);
        assertThrows(IllegalStateException.class, r::initializeAndAlign);
        assertTrue(h.fallbackStops > 0);
        r.close(); assertEquals(1, h.closes); h.assertZero();
    }

    @Test public void liveAutonomousCapIsEnforcedWithoutRestrictingTeleop() {
        double old = SwerveTuning.STEERING_MAX_COMMAND;
        try {
            for (boolean auto : new boolean[] {false, true}) {
                SwerveTuning.STEERING_MAX_COMMAND = old;
                Host h = new Host(); DifferentialSwerveRuntime r = ready(h, auto);
                DifferentialSwerveDrivetrain d = new DifferentialSwerveDrivetrain(r, auto);
                SwerveTuning.STEERING_MAX_COMMAND = .4;
                r.beginCycle();
                if (auto) assertThrows(IllegalArgumentException.class,
                        () -> d.drive(new DrivePowers(.5, .2, 0), false));
                else d.drive(new DrivePowers(.5, .2, 0), true);
                r.close(); h.assertZero();
            }
        } finally { SwerveTuning.STEERING_MAX_COMMAND = old; }
    }

    @Test public void cleanupRetainsEveryFailure() {
        RuntimeException first = new RuntimeException("first"), second = new RuntimeException("second");
        int[] attempts = {0};
        RuntimeException actual = assertThrows(RuntimeException.class, () -> Cleanup.runAll(
                () -> { throw first; }, () -> { throw second; }, () -> attempts[0]++));
        assertSame(first, actual); assertSame(second, actual.getSuppressed()[0]); assertEquals(1, attempts[0]);
    }

    @Test public void timingBoundaryIsInclusiveAndNegativeTimeFails() {
        assertEquals(.25, DifferentialSwerveRuntime.validateSampleSeconds(1, 250_000_001), 1e-12);
        assertThrows(HubSnapshotReader.FeedbackFault.class,
                () -> DifferentialSwerveRuntime.validateSampleSeconds(2, 1));
        assertThrows(HubSnapshotReader.FeedbackFault.class,
                () -> DifferentialSwerveRuntime.validateSnapshotAge(1, 250_000_002));
    }

    private static DifferentialSwerveRuntime ready(Host h, boolean auto) {
        DifferentialSwerveRuntime r = h.runtime(auto); r.initializeAndAlign(); r.arm(); return r;
    }
    private static double volts(boolean right, double cwDegrees) {
        return ((right ? SwervePodEncoder.RIGHT_FORWARD_DEGREES : SwervePodEncoder.LEFT_FORWARD_DEGREES)
                - cwDegrees + 360) % 360 / 360 * SwervePodEncoder.FULL_SCALE_VOLTS;
    }
    private static final class Motor implements DifferentialPod.MotorIO {
        double velocity; int zeroWrites, nonzeroWrites; boolean failZero, failNonzero;
        Runnable afterWrite;
        @Override public void setVelocity(double value) {
            if (value == 0) { zeroWrites++; if (failZero) throw new RuntimeException("zero failure"); }
            else { nonzeroWrites++; if (failNonzero) throw new RuntimeException("write failure"); }
            velocity = value;
            if (value != 0 && afterWrite != null) afterWrite.run();
        }
        @Override public double getVelocity() { return velocity; }
        @Override public int getPosition() { return 0; }
        @Override public void setBrake() { }
        @Override public void setFloat() { }
    }
    private static final class Host implements DifferentialSwerveRuntime.Host {
        final Motor[] motors = {new Motor(), new Motor(), new Motor(), new Motor()};
        final double[] volts = {volts(false, 0), volts(true, 0)};
        final int[] counts = {0, 0};
        long now = 1_000_000_000L;
        boolean stopped, started, failInit;
        int failReads, closes, fallbackStops;
        DifferentialSwerveRuntime runtime(boolean auto) { return new DifferentialSwerveRuntime(this, auto, () -> now); }
        @Override public DifferentialPod[] initialize(DifferentialPod.OutputGate gate, double maxDrive, double cap) {
            if (failInit) throw new IllegalStateException("partial acquisition");
            return new DifferentialPod[] {
                new DifferentialPod("left", false, motors[0], motors[1], () -> counts[0], () -> volts[0], maxDrive, cap, gate),
                new DifferentialPod("right", true, motors[2], motors[3], () -> counts[1], () -> volts[1], maxDrive, cap, gate)};
        }
        @Override public String refreshHubs() { now += 20_000_000L; return failReads-- > 0 ? "injected hub" : null; }
        @Override public void stopAcquiredMotors() {
            fallbackStops++;
            Cleanup.runAll(() -> motors[0].setVelocity(0), () -> motors[1].setVelocity(0),
                    () -> motors[2].setVelocity(0), () -> motors[3].setVelocity(0));
        }
        @Override public void applyMotorPidfIfChanged() { SwerveTuning.validate(); }
        @Override public boolean stopRequested() { return stopped; }
        @Override public boolean started() { return started; }
        @Override public void pause() { now += 10_000_000L; }
        @Override public void idle() { }
        @Override public void showAlignment(DifferentialPod left, DifferentialPod right) { }
        @Override public void showHealth(int recovered, String failure) { }
        @Override public void log(String message) { }
        @Override public void close() { closes++; stopAcquiredMotors(); }
        void assertZero() { for (Motor motor : motors) assertEquals(0, motor.velocity, 0); }
    }
}
