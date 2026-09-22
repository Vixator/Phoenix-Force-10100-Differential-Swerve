package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Collections;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class SafePedroFollowerTest {
    @Test public void bothUpdateOverloadsUseGuardAndDeliverManualDrive() {
        FakeLocalizer localizer = new FakeLocalizer();
        FakeDrivetrain drivetrain = new FakeDrivetrain();
        FakeGuard guard = new FakeGuard();
        SafePedroFollower follower = new SafePedroFollower(
                localizer, drivetrain, new ManualOnlyAlgorithm(), guard);
        follower.manual(0.2, -0.1, 0.05);
        follower.update();
        follower.update(0.03);
        assertEquals(2, guard.begins);
        assertEquals(2, guard.finishes);
        assertEquals(2, localizer.updates);
        assertEquals(2, drivetrain.drives);
    }

    @Test public void localizationFailureStopsImmediatelyAndLatches() {
        FakeLocalizer localizer = new FakeLocalizer();
        localizer.fail = true;
        FakeDrivetrain drivetrain = new FakeDrivetrain();
        FakeGuard guard = new FakeGuard();
        SafePedroFollower follower = new SafePedroFollower(
                localizer, drivetrain, new ManualOnlyAlgorithm(), guard);
        follower.manual(0.2, 0, 0);
        try { follower.update(); } catch (RuntimeException expected) { }
        assertEquals(0, drivetrain.drives);
        assertTrue(drivetrain.stops > 0);
        assertTrue(guard.fault);
    }

    @Test public void stopSendsHardwareStopWithoutWaitingForAnotherUpdate() {
        FakeDrivetrain drivetrain = new FakeDrivetrain();
        SafePedroFollower follower = new SafePedroFollower(
                new FakeLocalizer(), drivetrain, new ManualOnlyAlgorithm(), new FakeGuard());
        follower.manual(0.2, 0, 0);
        follower.stop();
        assertEquals(1, drivetrain.stops);
        assertTrue(follower.idle());
    }

    @Test public void latchedFaultCannotBeBypassedWithAnotherUpdate() {
        FakeDrivetrain d = new FakeDrivetrain(); FakeGuard g = new FakeGuard(); g.fault = true;
        SafePedroFollower f = new SafePedroFollower(new FakeLocalizer(), d, new ManualOnlyAlgorithm(), g);
        f.manual(.2, 0, 0);
        org.junit.Assert.assertThrows(IllegalStateException.class, f::update);
        assertEquals(0, g.begins); assertEquals(0, d.drives); assertTrue(d.stops > 0);
    }

    @Test public void invalidExplicitOrMeasuredTimeStopsWithoutDriving() {
        for (double time : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            FakeDrivetrain d = new FakeDrivetrain(); FakeGuard g = new FakeGuard();
            SafePedroFollower f = new SafePedroFollower(new FakeLocalizer(), d, new ManualOnlyAlgorithm(), g);
            f.manual(.2, 0, 0);
            org.junit.Assert.assertThrows(IllegalArgumentException.class, () -> f.update(time));
            assertEquals(0, d.drives); assertTrue(g.fault);
        }
        FakeDrivetrain d = new FakeDrivetrain(); FakeGuard g = new FakeGuard(); g.delta = .251;
        SafePedroFollower f = new SafePedroFollower(new FakeLocalizer(), d, new ManualOnlyAlgorithm(), g);
        org.junit.Assert.assertThrows(IllegalArgumentException.class, f::update);
        assertEquals(1, g.finishes); assertEquals(0, d.drives);
    }

    @Test public void cycleFinalizationFailureIsReportedAndStopsFollower() {
        FakeDrivetrain d = new FakeDrivetrain(); FakeGuard g = new FakeGuard(); g.failFinish = true;
        SafePedroFollower f = new SafePedroFollower(new FakeLocalizer(), d, new ManualOnlyAlgorithm(), g);
        f.manual(.2, 0, 0);
        org.junit.Assert.assertThrows(IllegalStateException.class, f::update);
        assertTrue(f.idle()); assertTrue(g.fault); assertTrue(d.stops > 0);
    }

    @Test public void foresightRunsFiniteAtNonzeroHeadingAndCompletesWithImmediateStop() {
        FakeLocalizer l = new FakeLocalizer(); FakeDrivetrain d = new FakeDrivetrain(); FakeGuard g = new FakeGuard();
        Pose start = new Pose(0, 0, Math.PI / 2), end = new Pose(0, 12, Math.PI / 2);
        l.setPose(start);
        SafePedroFollower f = new SafePedroFollower(l, d,
                new com.pedropathing.algorithm.Foresight(PedroFollowerConfig.syntheticForTests()), g);
        f.holdEnd.set(false);
        f.follow(com.pedropathing.api.Paths.line(start, end).constant(start));
        f.update();
        assertTrue(d.last.forward() > 0); assertEquals(0, d.last.strafe(), 1e-9);
        l.setPose(end); f.update(); f.update();
        assertTrue(f.idle()); assertTrue(d.stops > 0);
    }

    private static final class FakeGuard implements SafePedroFollower.CycleGuard {
        int begins;
        int finishes;
        boolean fault, failFinish;
        double delta = .02;
        @Override public double beginCycle() { begins++; return delta; }
        @Override public void finishCycle() { finishes++; if (failFinish) throw new IllegalStateException("finish failure"); }
        @Override public void latchFault(String message) { fault = true; }
        @Override public boolean hasFault() { return fault; }
    }

    private static final class FakeLocalizer implements Localizer {
        MotionState state = MotionState.zero();
        int updates;
        boolean fail;
        @Override public void setPose(Pose pose) { state = state.withPose(pose); }
        @Override public MotionState state() { return state; }
        @Override public void update() {
            updates++;
            if (fail) throw new GuardedPinpointLocalizer.LocalizationFault("injected");
        }
        @Override public void reset() { state = MotionState.zero(); }
    }

    private static final class FakeDrivetrain implements Drivetrain {
        DrivePowers last;
        int drives;
        int stops;
        @Override public void drive(DrivePowers powers, boolean manual) { PedroSwerveMath.requireFinite(powers, "test"); last = powers; drives++; }
        @Override public double maxScaling(DrivePowers current, DrivePowers delta) { return PedroSwerveMath.maxScaling(current, delta, .2); }
        @Override public void stop() { stops++; }
        @Override public void stop(boolean brake) { stops++; }
        @Override public Map<String, Object> debug() { return Collections.emptyMap(); }
        @Override public double interpolateVelocity(double x, double y, double theta) { return x; }
    }
}
