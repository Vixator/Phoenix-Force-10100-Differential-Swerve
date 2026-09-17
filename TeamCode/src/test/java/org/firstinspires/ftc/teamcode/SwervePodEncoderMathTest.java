package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class SwervePodEncoderMathTest {
    private static final double EPSILON = 1e-10;

    @Test
    public void voltageMapsAcrossTheFull3Point2VoltRange() {
        double[] volts = {0.0, 0.8, 1.6, 2.4, 3.2};
        for (int i = 0; i < volts.length; i++) {
            assertTrue(SwervePodEncoder.validVoltage(volts[i]));
            assertEquals(i * 90.0, SwervePodEncoder.rawDegrees(volts[i]), EPSILON);
        }
        assertEquals(0.0, SwervePodEncoder.absoluteRadians(3.2, 0.0, 1), EPSILON);
    }

    @Test
    public void absoluteOffsetsWrapWithBothAnalogSigns() {
        for (int sign : new int[]{1, -1}) {
            assertEquals(Math.toRadians(sign * 20.0),
                    SwervePodEncoder.absoluteRadians(voltsForDegrees(10.0), 350.0, sign), EPSILON);
            assertEquals(Math.toRadians(sign * -20.0),
                    SwervePodEncoder.absoluteRadians(voltsForDegrees(350.0), 10.0, sign), EPSILON);
            assertEquals(0.0,
                    SwervePodEncoder.absoluteRadians(voltsForDegrees(123.0), 123.0, sign), EPSILON);
        }
    }

    @Test
    public void smallRealStartupDeviationIsNotZeroed() {
        for (int sign : new int[]{1, -1}) {
            for (double deviation : new double[]{-0.25, 0.25}) {
                SwervePodEncoder encoder = new SwervePodEncoder(1);
                encoder.seed(SwervePodEncoder.absoluteRadians(
                        voltsForDegrees(123.0 + deviation), 123.0, sign), 700);
                encoder.update(700, 0.02);
                assertEquals(Math.toRadians(sign * deviation), encoder.getAngleRadians(), EPSILON);
                assertEquals(0.0, encoder.getRateRadiansPerSecond(), EPSILON);
            }
        }
    }

    @Test
    public void independentForwardReferencesAbsorb180DegreeMountingWithoutHandednessFlip() {
        for (int analogSign : new int[]{1, -1}) {
            for (double offset : new double[]{0.0, -15.0, 15.0}) {
                double left = SwervePodEncoder.absoluteRadians(
                        voltsForDegrees(40.0 + offset), 40.0, analogSign);
                double right = SwervePodEncoder.absoluteRadians(
                        voltsForDegrees(220.0 + offset), 220.0, analogSign);
                assertEquals(Math.toRadians(analogSign * offset), left, EPSILON);
                assertEquals(left, right, EPSILON);

                SwervePodEncoder leftEncoder = new SwervePodEncoder(1);
                SwervePodEncoder rightEncoder = new SwervePodEncoder(1);
                leftEncoder.seed(left, 100);
                rightEncoder.seed(right, -500);
                leftEncoder.update(132, 0.1);
                rightEncoder.update(-468, 0.1);
                assertEquals(left + Math.PI / 16.0, leftEncoder.getAngleRadians(), EPSILON);
                assertEquals(leftEncoder.getAngleRadians(), rightEncoder.getAngleRadians(), EPSILON);
                assertEquals(leftEncoder.getRateRadiansPerSecond(),
                        rightEncoder.getRateRadiansPerSecond(), EPSILON);
            }
        }
    }

    @Test
    public void wrappingUsesSignedAnglesAndPositivePiAtTheBoundary() {
        double[] degrees = {-720.0, -540.0, -190.0, -180.0, -10.0, 0.0, 180.0, 190.0, 540.0, 720.0};
        double[] expected = {0.0, 180.0, 170.0, 180.0, -10.0, 0.0, 180.0, -170.0, 180.0, 0.0};
        for (int i = 0; i < degrees.length; i++) {
            assertEquals(Math.toRadians(expected[i]),
                    SwervePodEncoder.wrapRadians(Math.toRadians(degrees[i])), EPSILON);
        }
    }

    @Test
    public void invalidVoltagesAreRejected() {
        for (double volts : new double[]{-0.001, 3.200001, Double.NaN,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY}) {
            assertFalse(SwervePodEncoder.validVoltage(volts));
            assertThrows(IllegalArgumentException.class, () -> SwervePodEncoder.rawDegrees(volts));
            assertThrows(IllegalArgumentException.class,
                    () -> SwervePodEncoder.absoluteRadians(volts, 90.0, 1));
        }
    }

    @Test
    public void invalidForwardReferencesAndSensorSignsAreRejected() {
        for (double forward : new double[]{-0.001, 360.0, 720.0, Double.NaN,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class,
                    () -> SwervePodEncoder.absoluteRadians(1.6, forward, 1));
        }
        for (int sign : new int[]{0, -2, 2, Integer.MIN_VALUE, Integer.MAX_VALUE}) {
            assertThrows(IllegalArgumentException.class, () -> new SwervePodEncoder(sign));
            assertThrows(IllegalArgumentException.class,
                    () -> SwervePodEncoder.absoluteRadians(1.6, 90.0, sign));
        }
    }

    @Test
    public void invalidSeedAnglesLeaveTrackingStateUnchanged() {
        SwervePodEncoder encoder = new SwervePodEncoder(1);
        encoder.seed(0.0, 100);
        encoder.update(356, 0.5);
        for (double angle : new double[]{Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class, () -> encoder.seed(angle, 999));
            assertEquals(356, encoder.getCount());
            assertEquals(Math.PI / 2.0, encoder.getAngleRadians(), EPSILON);
            assertEquals(Math.PI, encoder.getRateRadiansPerSecond(), EPSILON);
        }
    }

    @Test
    public void invalidSampleTimesLeaveTrackingStateUnchanged() {
        SwervePodEncoder encoder = new SwervePodEncoder(1);
        encoder.seed(0.0, 100);
        encoder.update(356, 0.5);
        for (double seconds : new double[]{0.0, -0.01, Double.NaN,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class, () -> encoder.update(612, seconds));
            assertEquals(356, encoder.getCount());
            assertEquals(Math.PI / 2.0, encoder.getAngleRadians(), EPSILON);
            assertEquals(Math.PI, encoder.getRateRadiansPerSecond(), EPSILON);
        }
        encoder.update(612, 0.5);
        assertEquals(Math.PI, encoder.getAngleRadians(), EPSILON);
        assertEquals(Math.PI, encoder.getRateRadiansPerSecond(), EPSILON);
    }

    @Test
    public void countUpdatesAndRatesRespectBothQuadratureSigns() {
        for (int sign : new int[]{1, -1}) {
            SwervePodEncoder encoder = new SwervePodEncoder(sign);
            encoder.seed(0.0, 100);
            encoder.update(356, 0.5);
            assertEquals(356, encoder.getCount());
            assertEquals(sign * Math.PI / 2.0, encoder.getAngleRadians(), EPSILON);
            assertEquals(sign * Math.PI, encoder.getRateRadiansPerSecond(), EPSILON);
            encoder.update(228, 0.25);
            assertEquals(228, encoder.getCount());
            assertEquals(sign * Math.PI / 4.0, encoder.getAngleRadians(), EPSILON);
            assertEquals(-sign * Math.PI, encoder.getRateRadiansPerSecond(), EPSILON);
            encoder.update(228, 0.1);
            assertEquals(sign * Math.PI / 4.0, encoder.getAngleRadians(), EPSILON);
            assertEquals(0.0, encoder.getRateRadiansPerSecond(), EPSILON);
        }
    }

    @Test
    public void signedCountRolloverWorksInBothDirectionsAndWithBothSigns() {
        for (int sign : new int[]{1, -1}) {
            SwervePodEncoder encoder = new SwervePodEncoder(sign);
            encoder.seed(0.0, Integer.MAX_VALUE - 2);
            encoder.update(Integer.MIN_VALUE + 2, 0.25);
            double fiveTicks = sign * 5.0 * 2.0 * Math.PI / 1024.0;
            assertEquals(Integer.MIN_VALUE + 2, encoder.getCount());
            assertEquals(fiveTicks, encoder.getAngleRadians(), EPSILON);
            assertEquals(fiveTicks / 0.25, encoder.getRateRadiansPerSecond(), EPSILON);
            encoder.update(Integer.MAX_VALUE - 2, 0.5);
            assertEquals(Integer.MAX_VALUE - 2, encoder.getCount());
            assertEquals(0.0, encoder.getAngleRadians(), EPSILON);
            assertEquals(-fiveTicks / 0.5, encoder.getRateRadiansPerSecond(), EPSILON);
        }
    }

    @Test
    public void seedUsesExistingCountWithoutAFirstTickJump() {
        SwervePodEncoder encoder = new SwervePodEncoder(-1);
        encoder.seed(Math.toRadians(190.0), -123456);
        assertEquals(-123456, encoder.getCount());
        assertEquals(Math.toRadians(-170.0), encoder.getAngleRadians(), EPSILON);
        assertEquals(0.0, encoder.getRateRadiansPerSecond(), EPSILON);
        encoder.update(-123456, 0.02);
        assertEquals(Math.toRadians(-170.0), encoder.getAngleRadians(), EPSILON);
        assertEquals(0.0, encoder.getRateRadiansPerSecond(), EPSILON);
        encoder.update(-123455, 0.02);
        assertEquals(Math.toRadians(-170.0) - 2.0 * Math.PI / 1024.0,
                encoder.getAngleRadians(), EPSILON);
    }

    @Test
    public void reseedingResetsRateAndReplacesCountBaseline() {
        SwervePodEncoder encoder = new SwervePodEncoder(1);
        encoder.seed(0.0, 0);
        encoder.update(256, 0.5);
        assertEquals(Math.PI, encoder.getRateRadiansPerSecond(), EPSILON);
        encoder.seed(Math.toRadians(-30.0), -1000);
        assertEquals(-1000, encoder.getCount());
        assertEquals(Math.toRadians(-30.0), encoder.getAngleRadians(), EPSILON);
        assertEquals(0.0, encoder.getRateRadiansPerSecond(), EPSILON);
        encoder.update(-1000, 0.1);
        assertEquals(Math.toRadians(-30.0), encoder.getAngleRadians(), EPSILON);
        assertEquals(0.0, encoder.getRateRadiansPerSecond(), EPSILON);
        encoder.update(-744, 0.5);
        assertEquals(Math.toRadians(60.0), encoder.getAngleRadians(), EPSILON);
        assertEquals(Math.PI, encoder.getRateRadiansPerSecond(), EPSILON);
    }

    @Test
    public void runtimeQuadratureAccumulatesRelativeToInitialAnalogAngle() {
        for (int analogSign : new int[]{1, -1}) {
            for (int quadratureSign : new int[]{1, -1}) {
                SwervePodEncoder encoder = new SwervePodEncoder(quadratureSign);
                encoder.seed(SwervePodEncoder.absoluteRadians(
                        voltsForDegrees(200.0), 30.0, analogSign), 1000);
                encoder.update(1256, 0.5);
                double expectedDegrees = analogSign * 170.0 + quadratureSign * 90.0;
                if (expectedDegrees > 180.0) expectedDegrees -= 360.0;
                if (expectedDegrees <= -180.0) expectedDegrees += 360.0;
                assertEquals(Math.toRadians(expectedDegrees), encoder.getAngleRadians(), EPSILON);
                encoder.update(2280, 0.25);
                assertEquals(2280, encoder.getCount());
                assertEquals(Math.toRadians(expectedDegrees), encoder.getAngleRadians(), EPSILON);
                // A whole turn has zero wrapped displacement but nonzero angular velocity.
                assertEquals(quadratureSign * 8.0 * Math.PI, encoder.getRateRadiansPerSecond(), EPSILON);
                encoder.update(2024, 0.5);
                assertEquals(Math.toRadians(analogSign * 170.0), encoder.getAngleRadians(), EPSILON);
                assertEquals(-quadratureSign * Math.PI, encoder.getRateRadiansPerSecond(), EPSILON);
            }
        }
    }

    @Test
    public void calibrationIsNotReadyBeforeMeasurementsAreVerified() {
        assertFalse(SwervePodEncoder.CALIBRATION_VERIFIED);
        assertFalse(SwervePodEncoder.calibrationReady());
    }

    private static double voltsForDegrees(double degrees) {
        return degrees / 360.0 * 3.2;
    }
}
