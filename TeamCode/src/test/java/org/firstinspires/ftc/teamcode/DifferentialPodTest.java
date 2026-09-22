package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class DifferentialPodTest {
    private static final double EPS = 1e-9;

    @Test public void leftAndRightMappingsApplyScaleExactlyOnce() {
        FakeMotor leftA = new FakeMotor();
        FakeMotor leftB = new FakeMotor();
        DifferentialPod left = pod(false, leftA, leftB, new Gate(true));
        left.move(PedroSwerveMath.wheelAngle(0.0), 1.0, false);
        double expected = PedroDriveConfig.AUTONOMOUS_MAX_DRIVE
                * HardwareConstants.MAX_MOTOR_TICKS_PER_SECOND;
        assertEquals(expected, leftA.velocity, EPS);
        assertEquals(expected, leftB.velocity, EPS);

        FakeMotor rightA = new FakeMotor();
        FakeMotor rightB = new FakeMotor();
        DifferentialPod right = pod(true, rightA, rightB, new Gate(true));
        right.move(PedroSwerveMath.wheelAngle(0.0), 1.0, false);
        assertEquals(-expected, rightA.velocity, EPS);
        assertEquals(-expected, rightB.velocity, EPS);
    }

    @Test public void rightMappingNegatesAndSwapsMixedCommands() {
        double[] mapped = DifferentialPod.hardwareTargets(true, 0.2, -0.1);
        assertEquals(0.1 * HardwareConstants.MAX_MOTOR_TICKS_PER_SECOND, mapped[0], EPS);
        assertEquals(-0.2 * HardwareConstants.MAX_MOTOR_TICKS_PER_SECOND, mapped[1], EPS);
    }

    @Test public void ignoreAngleChangesClearsSteeringSlewAndZeroReallyStops() {
        FakeMotor a = new FakeMotor();
        FakeMotor b = new FakeMotor();
        DifferentialPod pod = pod(false, a, b, new Gate(true));
        pod.move(PedroSwerveMath.wheelAngle(Math.toRadians(45)), 0.5, false);
        assertTrue(Math.abs((double) pod.debug().get("steeringCommand")) > 0.0);
        pod.move(PedroSwerveMath.wheelAngle(Math.toRadians(45)), 0.5, true);
        assertEquals(0.0, (double) pod.debug().get("steeringCommand"), EPS);
        pod.move(PedroSwerveMath.wheelAngle(Math.toRadians(45)), 0.0, true);
        assertEquals(0.0, a.velocity, EPS);
        assertEquals(0.0, b.velocity, EPS);
    }

    @Test public void safeZeroAttemptsSecondMotorAfterFirstFailure() {
        FakeMotor a = new FakeMotor();
        FakeMotor b = new FakeMotor();
        DifferentialPod pod = pod(false, a, b, new Gate(true));
        a.failNextZero = true;
        try { pod.safeZero(); } catch (RuntimeException expected) { }
        assertEquals(1, b.zeroWrites);
    }

    @Test public void floatIsRejectedWhileArmedAndBrakeReachesBothMotors() {
        FakeMotor a = new FakeMotor();
        FakeMotor b = new FakeMotor();
        DifferentialPod pod = pod(false, a, b, new Gate(true));
        try {
            pod.setToFloat();
            fail("Armed FLOAT must be rejected");
        } catch (IllegalStateException expected) {
            assertEquals(0.0, a.velocity, EPS);
            assertEquals(0.0, b.velocity, EPS);
        }
        pod.setToBreak();
        assertEquals(1, a.brakeWrites);
        assertEquals(1, b.brakeWrites);
    }

    private static DifferentialPod pod(boolean right, FakeMotor a, FakeMotor b, Gate gate) {
        double volts = (right ? SwervePodEncoder.RIGHT_FORWARD_DEGREES
                : SwervePodEncoder.LEFT_FORWARD_DEGREES) / 360.0 * SwervePodEncoder.FULL_SCALE_VOLTS;
        return new DifferentialPod(right ? "right" : "left", right, a, b,
                () -> 0, () -> volts, PedroDriveConfig.AUTONOMOUS_MAX_DRIVE, PedroDriveConfig.MAX_COMBINED_MOTOR_COMMAND, gate);
    }

    private static final class Gate implements DifferentialPod.OutputGate {
        boolean armed;
        Gate(boolean armed) { this.armed = armed; }
        @Override public void requireOutputAllowed() {
            if (!armed) throw new IllegalStateException("not armed");
        }
        @Override public void latchFault(String message) { armed = false; }
        @Override public boolean allowMaintenanceFloat() { return false; }
        @Override public double snapshotAgeMillis() { return 0; }
        @Override public String faultDescription() { return "none"; }
    }

    private static final class FakeMotor implements DifferentialPod.MotorIO {
        double velocity;
        int zeroWrites;
        int brakeWrites;
        boolean failNextZero;
        @Override public void setVelocity(double value) {
            if (value == 0.0) {
                zeroWrites++;
                if (failNextZero) {
                    failNextZero = false;
                    throw new RuntimeException("injected zero failure");
                }
            }
            velocity = value;
        }
        @Override public double getVelocity() { return velocity; }
        @Override public int getPosition() { return 0; }
        @Override public void setBrake() { brakeWrites++; }
        @Override public void setFloat() { }
    }
}
