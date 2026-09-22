package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointConfig;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.junit.Test;

public class GuardedPinpointLocalizerTest {
    @Test public void finiteFieldFrameStateIsAccepted() {
        GuardedPinpointLocalizer.validateState(MotionState.ofVelocity(
                new Pose(1, 2, Math.PI / 2), new Velocity(-3, 4, 0.5)));
    }

    @Test(expected = GuardedPinpointLocalizer.LocalizationFault.class)
    public void nonfiniteVelocityIsRejected() {
        GuardedPinpointLocalizer.validateState(MotionState.ofVelocity(
                Pose.zero(), new Velocity(Double.NaN, 0, 0)));
    }

    @Test public void diagnosticConfigUsesInchesAndExplicitCandidateDirections() {
        PinpointConfig config = PinpointSettings.diagnosticConfig();
        org.junit.Assert.assertEquals(PinpointSettings.NAME, config.name.get());
        org.junit.Assert.assertEquals(DistanceUnit.INCH, config.globalDistanceUnit.get());
        org.junit.Assert.assertEquals(DistanceUnit.MM, config.offsetUnits.get());
        org.junit.Assert.assertEquals(PinpointSettings.X_DIRECTION_CANDIDATE, config.xPodDirection.get());
        org.junit.Assert.assertEquals(PinpointSettings.Y_DIRECTION_CANDIDATE, config.yPodDirection.get());
    }

    @Test
    public void poweredConfigUsesCommissionedFrame() {
        PinpointConfig config = PinpointSettings.poweredConfig();
        org.junit.Assert.assertEquals(PinpointSettings.HEADING_CONVENTION,
                PinpointSettings.HeadingConvention.COUNTERCLOCKWISE_POSITIVE);
        org.junit.Assert.assertEquals(PinpointSettings.NAME, config.name.get());
    }

    @Test public void updateReadsExactlyOnceAndPreservesFieldFrame() {
        Probe p = new Probe();
        p.state = MotionState.ofVelocity(new Pose(2, 4, Math.PI / 2), new Velocity(-3, 4, .5));
        GuardedPinpointLocalizer localizer = p.localizer();
        localizer.update();
        org.junit.Assert.assertEquals(1, p.updates);
        org.junit.Assert.assertEquals(-3, localizer.velocity().vx, 0);
        org.junit.Assert.assertEquals(4, localizer.twist().vx, 1e-9);
        org.junit.Assert.assertEquals(3, localizer.twist().vy, 1e-9);
    }

    @Test public void notReadyTransactionDelayAndNonfiniteFeedbackAreRejected() {
        Probe p = new Probe(); GuardedPinpointLocalizer l = p.localizer();
        p.status = com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        org.junit.Assert.assertThrows(GuardedPinpointLocalizer.LocalizationFault.class, l::update);
        p.status = com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.READY;
        p.delay = 251_000_000L;
        org.junit.Assert.assertThrows(HubSnapshotReader.FeedbackFault.class, l::update);
        p.delay = 1;
        p.state = MotionState.ofVelocity(new Pose(Double.NaN, 0, 0), Velocity.zero());
        org.junit.Assert.assertThrows(GuardedPinpointLocalizer.LocalizationFault.class, l::update);
    }

    @Test public void calibrationPollsSingleDelegateAndArmedResetIsRejected() {
        Probe p = new Probe(); GuardedPinpointLocalizer l = p.localizer();
        l.calibrateStationary(() -> false, () -> p.now += 10_000_000L, true);
        org.junit.Assert.assertEquals(1, p.calibrations);
        org.junit.Assert.assertEquals(1, p.updates);
        p.armed = true;
        org.junit.Assert.assertThrows(IllegalStateException.class, l::reset);
        org.junit.Assert.assertThrows(IllegalStateException.class,
                () -> l.calibrateStationary(() -> false, () -> {}, true));
        org.junit.Assert.assertEquals(0, p.resets);
    }

    @Test public void calibrationTimeoutAndStopCancelWithoutReady() {
        Probe p = new Probe(); GuardedPinpointLocalizer l = p.localizer();
        p.status = com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        org.junit.Assert.assertThrows(GuardedPinpointLocalizer.LocalizationFault.class,
                () -> l.calibrateStationary(() -> false, () -> p.now += 100_000_000L, true));
        int before = p.updates;
        org.junit.Assert.assertThrows(GuardedPinpointLocalizer.LocalizationFault.class,
                () -> l.calibrateStationary(() -> true, () -> {}, true));
        org.junit.Assert.assertEquals(before, p.updates);
    }

    @Test public void setPoseDoesNotResetImuAndRejectsInvalidPose() {
        Probe p = new Probe(); GuardedPinpointLocalizer l = p.localizer();
        l.setPose(new Pose(5, 6, .7));
        org.junit.Assert.assertEquals(5, l.pose().x(), 0);
        org.junit.Assert.assertEquals(0, p.resets);
        org.junit.Assert.assertThrows(IllegalArgumentException.class,
                () -> l.setPose(new Pose(0, 0, Double.NaN)));
    }

    private static final class Probe implements com.pedropathing.localization.Localizer,
            GuardedPinpointLocalizer.Sensor {
        MotionState state = MotionState.zero();
        long now = 1_000_000_000L, delay = 1_000_000L;
        int updates, resets, calibrations;
        boolean armed;
        com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus status =
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.READY;
        GuardedPinpointLocalizer localizer() { return new GuardedPinpointLocalizer(this, this, () -> armed, () -> now); }
        @Override public void update() { updates++; now += delay; }
        @Override public MotionState state() { return state; }
        @Override public void setPose(Pose pose) { state = state.withPose(pose); }
        @Override public void reset() { resets++; }
        @Override public com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus status() { return status; }
        @Override public void calibrate(boolean reset) { calibrations++; }
        @Override public double loopMicros() { return 1000; }
        @Override public double frequencyHz() { return 1000; }
    }
}
