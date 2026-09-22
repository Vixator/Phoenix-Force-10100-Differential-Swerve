package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.math.Vector2D;

import org.junit.Test;

import java.util.Collections;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class PedroSwerveMathTest {
    private static final double EPS = 1e-10;

    @Test public void frameConversionsHaveDocumentedSigns() {
        assertEquals(-0.4, PedroSwerveMath.rightFromPedroStrafe(0.4), EPS);
        assertTrue(PedroSwerveMath.clockwiseRadiansPerSecond(0.2) < 0.0);
        assertEquals(0.0, PedroSwerveMath.encoderAngle(Math.PI / 2.0), EPS);
        assertEquals(-Math.PI / 2.0, PedroSwerveMath.encoderAngle(Math.PI), EPS);
        assertEquals(Math.PI / 2.0, PedroSwerveMath.wheelAngle(0.0), EPS);
    }

    @Test public void linearWheelVectorsMatchTwoPodGeometry() {
        Vector2D[] pureTurn = PedroSwerveMath.linearWheelVectors(new DrivePowers(0, 0, 0.2));
        assertEquals(-0.2, pureTurn[0].x(), EPS);
        assertEquals(0.2, pureTurn[1].x(), EPS);
        Vector2D[] combined = PedroSwerveMath.linearWheelVectors(new DrivePowers(0.4, 0.3, -0.1));
        assertEquals(0.5, combined[0].x(), EPS);
        assertEquals(0.3, combined[0].y(), EPS);
        assertEquals(0.3, combined[1].x(), EPS);
    }

    @Test public void maxScalingHandlesZeroAndBoundaryDirection() {
        assertEquals(1.0, PedroSwerveMath.maxScaling(
                new DrivePowers(0.2, 0, 0), DrivePowers.zero(), 0.2), EPS);
        assertEquals(1.0, PedroSwerveMath.maxScaling(
                new DrivePowers(1, 0, 0), new DrivePowers(-0.5, 0, 0), 0.2), EPS);
        assertEquals(0.0, PedroSwerveMath.maxScaling(
                new DrivePowers(1, 0, 0), new DrivePowers(0.5, 0, 0), 0.2), EPS);
        assertEquals(0.5, PedroSwerveMath.maxScaling(
                DrivePowers.zero(), new DrivePowers(0, 0, 0.4), 0.2), EPS);
    }

    @Test public void maxScalingReturnsGreatestCombinedFeasibleStep() {
        DrivePowers current = new DrivePowers(0.4, 0.2, 0.05);
        DrivePowers delta = new DrivePowers(0.9, 0.7, 0.1);
        double lambda = PedroSwerveMath.maxScaling(current, delta, 0.2);
        assertTrue(lambda > 0.0 && lambda < 1.0);
        DrivePowers result = add(current, delta, lambda);
        for (Vector2D vector : PedroSwerveMath.linearWheelVectors(result)) {
            assertTrue(vector.magnitude() <= 1.0 + 1e-10);
        }
        assertTrue(Math.abs(result.turn()) <= 0.2 + EPS);
        DrivePowers beyond = add(current, delta, lambda + 1e-5);
        boolean outside = Math.abs(beyond.turn()) > 0.2
                || PedroSwerveMath.linearWheelVectors(beyond)[0].magnitude() > 1.0
                || PedroSwerveMath.linearWheelVectors(beyond)[1].magnitude() > 1.0;
        assertTrue(outside);
    }

    @Test(expected = IllegalArgumentException.class)
    public void maxScalingRejectsInfeasibleCurrentState() {
        PedroSwerveMath.maxScaling(new DrivePowers(1.1, 0, 0), DrivePowers.zero(), 0.2);
    }

    @Test public void velocityAndInheritedAccelerationInterpolationAreFinite() {
        assertEquals(10.0, PedroSwerveMath.interpolateVelocity(10, 20, 0), EPS);
        assertEquals(20.0, PedroSwerveMath.interpolateVelocity(10, 20, Math.PI / 2), EPS);
        Drivetrain drivetrain = new Drivetrain() {
            @Override public void drive(DrivePowers powers, boolean manual) { }
            @Override public double maxScaling(DrivePowers current, DrivePowers delta) { return 1; }
            @Override public void stop() { }
            @Override public void stop(boolean brake) { }
            @Override public Map<String, Object> debug() { return Collections.emptyMap(); }
            @Override public double interpolateVelocity(double x, double y, double theta) {
                return PedroSwerveMath.interpolateVelocity(x, y, theta);
            }
        };
        assertEquals(8.0, drivetrain.interpolateAcceleration(8, 12, 0), EPS);
        assertEquals(12.0, drivetrain.interpolateAcceleration(8, 12, Math.PI / 2), EPS);
    }

    @Test public void tinyOutwardDeltaAtBoundaryCannotExceedEnvelope() {
        assertEquals(0, PedroSwerveMath.maxScaling(new DrivePowers(1, 0, 0),
                new DrivePowers(1e-7, 0, 0), .2), 0);
    }

    private static DrivePowers add(DrivePowers a, DrivePowers b, double lambda) {
        return new DrivePowers(a.forward() + lambda * b.forward(),
                a.strafe() + lambda * b.strafe(), a.turn() + lambda * b.turn());
    }
}
