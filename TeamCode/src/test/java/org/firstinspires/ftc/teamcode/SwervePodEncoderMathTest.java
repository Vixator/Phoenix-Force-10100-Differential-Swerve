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
                assertEquals(left + radiansForCounts(32), leftEncoder.getAngleRadians(), EPSILON);
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
        for (double volts : new double[]{-0.001, 3.300001, Double.NaN,
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
            assertEquals(radiansForCounts(256), encoder.getAngleRadians(), EPSILON);
            assertEquals(radiansForCounts(256) / 0.5, encoder.getRateRadiansPerSecond(), EPSILON);
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
            assertEquals(radiansForCounts(256), encoder.getAngleRadians(), EPSILON);
            assertEquals(radiansForCounts(256) / 0.5, encoder.getRateRadiansPerSecond(), EPSILON);
        }
        encoder.update(612, 0.5);
        assertEquals(radiansForCounts(512), encoder.getAngleRadians(), EPSILON);
        assertEquals(radiansForCounts(256) / 0.5, encoder.getRateRadiansPerSecond(), EPSILON);
    }

    @Test
    public void countUpdatesAndRatesRespectBothQuadratureSigns() {
        for (int sign : new int[]{1, -1}) {
            SwervePodEncoder encoder = new SwervePodEncoder(sign);
            encoder.seed(0.0, 100);
            encoder.update(356, 0.5);
            assertEquals(356, encoder.getCount());
            assertEquals(sign * radiansForCounts(256), encoder.getAngleRadians(), EPSILON);
            assertEquals(sign * radiansForCounts(256) / 0.5, encoder.getRateRadiansPerSecond(), EPSILON);
            encoder.update(228, 0.25);
            assertEquals(228, encoder.getCount());
            assertEquals(sign * radiansForCounts(128), encoder.getAngleRadians(), EPSILON);
            assertEquals(-sign * radiansForCounts(128) / 0.25, encoder.getRateRadiansPerSecond(), EPSILON);
            encoder.update(228, 0.1);
            assertEquals(sign * radiansForCounts(128), encoder.getAngleRadians(), EPSILON);
            assertEquals(0.0, encoder.getRateRadiansPerSecond(), EPSILON);
        }
    }

    @Test
    public void signedCountRolloverWorksInBothDirectionsAndWithBothSigns() {
        for (int sign : new int[]{1, -1}) {
            SwervePodEncoder encoder = new SwervePodEncoder(sign);
            encoder.seed(0.0, Integer.MAX_VALUE - 2);
            encoder.update(Integer.MIN_VALUE + 2, 0.25);
            double fiveTicks = sign * radiansForCounts(5);
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
        assertEquals(Math.toRadians(-170.0) - radiansForCounts(1),
                encoder.getAngleRadians(), EPSILON);
    }

    @Test
    public void reseedingResetsRateAndReplacesCountBaseline() {
        SwervePodEncoder encoder = new SwervePodEncoder(1);
        encoder.seed(0.0, 0);
        encoder.update(256, 0.5);
        assertEquals(radiansForCounts(256) / 0.5, encoder.getRateRadiansPerSecond(), EPSILON);
        encoder.seed(Math.toRadians(-30.0), -1000);
        assertEquals(-1000, encoder.getCount());
        assertEquals(Math.toRadians(-30.0), encoder.getAngleRadians(), EPSILON);
        assertEquals(0.0, encoder.getRateRadiansPerSecond(), EPSILON);
        encoder.update(-1000, 0.1);
        assertEquals(Math.toRadians(-30.0), encoder.getAngleRadians(), EPSILON);
        assertEquals(0.0, encoder.getRateRadiansPerSecond(), EPSILON);
        encoder.update(-744, 0.5);
        assertEquals(SwervePodEncoder.wrapRadians(Math.toRadians(-30.0) + radiansForCounts(256)),
                encoder.getAngleRadians(), EPSILON);
        assertEquals(radiansForCounts(256) / 0.5, encoder.getRateRadiansPerSecond(), EPSILON);
    }

    @Test
    public void runtimeQuadratureUsesConfiguredCountScale() {
        for (int sign : new int[]{1, -1}) {
            SwervePodEncoder encoder = new SwervePodEncoder(sign);
            encoder.seed(0.0, 1000);
            encoder.update(2024, 0.5);
            assertEquals(sign * radiansForCounts(1024), encoder.getAngleRadians(), EPSILON);
            assertEquals(sign * radiansForCounts(1024) / 0.5,
                    encoder.getRateRadiansPerSecond(), EPSILON);
            encoder.update(5096, 0.5);
            assertEquals(0.0, encoder.getAngleRadians(), EPSILON);
            assertEquals(sign * radiansForCounts(3072) / 0.5,
                    encoder.getRateRadiansPerSecond(), EPSILON);
        }
    }

    @Test
    public void analogErrorUsesShortestSignedPathAndAnalogSign() {
        assertEquals(20.0,
                SwervePodEncoder.analogErrorDegrees(voltsForDegrees(350.0), 10.0, 1), EPSILON);
        assertEquals(-20.0,
                SwervePodEncoder.analogErrorDegrees(voltsForDegrees(10.0), 350.0, 1), EPSILON);
        assertEquals(-20.0,
                SwervePodEncoder.analogErrorDegrees(voltsForDegrees(350.0), 10.0, -1), EPSILON);
    }

    @Test
    public void degreeWrappingMatchesSteeringBoundaries() {
        assertEquals(180.0, SwervePodEncoder.wrapDegrees(-180.0), EPSILON);
        assertEquals(-170.0, SwervePodEncoder.wrapDegrees(190.0), EPSILON);
        assertEquals(170.0, SwervePodEncoder.wrapDegrees(-190.0), EPSILON);
    }

    @Test
    public void alignmentControllerCommandsShortestPathAndStopsAtTarget() {
        PodAlignmentController controller = new PodAlignmentController(10.0, 1);
        controller.start();
        controller.step(voltsForDegrees(200.0), 0.01);
        assertTrue(controller.isActive());
        assertEquals(-SwerveTuning.ALIGNMENT_MAX_COMMAND, controller.getCommand(), EPSILON);
        controller.step(voltsForDegrees(10.5), 0.01);
        assertTrue(controller.isActive());
        assertEquals(0.0, controller.getCommand(), EPSILON);
        controller.step(voltsForDegrees(10.5), PodAlignmentController.SETTLE_SECONDS);
        assertTrue(controller.isComplete());
        assertEquals(0.0, controller.getCommand(), EPSILON);
    }

    @Test
    public void alignmentControllerFailsOnInvalidFeedbackOrTimeout() {
        PodAlignmentController invalid = new PodAlignmentController(10.0, 1);
        invalid.start();
        invalid.step(-0.1, 0.01);
        assertTrue(invalid.isFailed());
        assertEquals(0.0, invalid.getCommand(), EPSILON);

        PodAlignmentController timeout = new PodAlignmentController(10.0, 1);
        timeout.start();
        timeout.step(voltsForDegrees(200.0), PodAlignmentController.TIMEOUT_SECONDS);
        assertTrue(timeout.isFailed());
        assertEquals(0.0, timeout.getCommand(), EPSILON);
    }

    @Test
    public void upperRailMarginClampsToWrapWithoutChangingCalibrationScale() {
        for (double volts : new double[]{3.2, 3.200001, 3.25, 3.3}) {
            assertTrue(SwervePodEncoder.validVoltage(volts));
            assertEquals(360.0, SwervePodEncoder.rawDegrees(volts), EPSILON);
            assertEquals(0.0, SwervePodEncoder.absoluteRadians(volts, 0.0, -1), EPSILON);
            assertEquals(Math.toRadians(SwervePodEncoder.RIGHT_FORWARD_DEGREES),
                    SwervePodEncoder.absoluteRadians(volts, SwervePodEncoder.RIGHT_FORWARD_DEGREES, -1), EPSILON);
        }
        assertEquals(180.0, SwervePodEncoder.rawDegrees(1.6), EPSILON);
        assertEquals(0.0, SwervePodEncoder.absoluteRadians(0.122, SwervePodEncoder.LEFT_FORWARD_DEGREES, -1), EPSILON);
        assertEquals(0.0, SwervePodEncoder.absoluteRadians(0.258, SwervePodEncoder.RIGHT_FORWARD_DEGREES, -1), EPSILON);
    }

    @Test
    public void alignmentCanCrossTheAnalogWrapWithAnUpperRailReading() {
        PodAlignmentController controller = new PodAlignmentController(SwervePodEncoder.RIGHT_FORWARD_DEGREES, -1);
        controller.start();
        controller.step(3.25, 0.02);
        assertTrue(controller.isActive());
        assertFalse(controller.isFailed());
        assertTrue(controller.getCommand() > 0.0); // CCW through the wrap toward .258 V.
        controller.step(0.01, 0.02);
        assertTrue(controller.getCommand() > 0.0);
        controller.step(0.258, 0.02);
        controller.step(0.258, PodAlignmentController.SETTLE_SECONDS);
        assertTrue(controller.isComplete());
        assertEquals(0.0, controller.getCommand(), EPSILON);
    }

    @Test
    public void outOfRangeAlignmentFaultReportsVoltageAndStopsCommand() {
        PodAlignmentController controller = new PodAlignmentController(SwervePodEncoder.RIGHT_FORWARD_DEGREES, -1);
        controller.start();
        controller.step(1.6, 0.02);
        assertTrue(Math.abs(controller.getCommand()) > 0.0);
        controller.step(3.4, 0.02);
        assertTrue(controller.isFailed());
        assertEquals(0.0, controller.getCommand(), EPSILON);
        assertTrue(controller.getStatus().contains("3.4 V"));
        assertTrue(controller.getStatus().contains("0..3.3 V"));
    }

    @Test
    public void alignmentMustStayInToleranceInsteadOfJustPassingThroughIt() {
        PodAlignmentController controller = new PodAlignmentController(10.0, -1);
        controller.start();
        controller.step(voltsForDegrees(10.0), 0.2);
        assertFalse(controller.isComplete()); // Time before the first in-window sample does not count.
        controller.step(voltsForDegrees(10.0), 0.06);
        controller.step(voltsForDegrees(20.0), 0.01); // Overshoot resets settling.
        assertTrue(Math.abs(controller.getCommand()) > 0.0);
        controller.step(voltsForDegrees(10.0), 0.01);
        controller.step(voltsForDegrees(10.0), 0.06);
        assertFalse(controller.isComplete());
        controller.step(voltsForDegrees(10.0), 0.05);
        assertTrue(controller.isComplete());
        assertEquals(0.0, controller.getCommand(), EPSILON);
        controller.abort("partner failed");
        assertTrue(controller.isFailed());
        assertFalse(controller.isComplete());
        controller.start();
        controller.step(voltsForDegrees(10.0), 0.01);
        assertTrue(controller.isActive()); // A new run must settle again.
    }

    @Test
    public void alignmentCannotRemainInSettlingStatePastTimeout() {
        PodAlignmentController controller = new PodAlignmentController(10.0, -1);
        controller.start();
        controller.step(voltsForDegrees(30.0), PodAlignmentController.TIMEOUT_SECONDS - 0.01);
        controller.step(voltsForDegrees(10.0), 0.02);
        assertTrue(controller.isFailed());
        assertEquals(0.0, controller.getCommand(), EPSILON);
    }

    @Test
    public void lateSettledReadingDoesNotOverrideAlignmentTimeout() {
        PodAlignmentController controller = new PodAlignmentController(10.0, -1);
        controller.start();
        controller.step(voltsForDegrees(10.0), PodAlignmentController.TIMEOUT_SECONDS - 0.05);
        controller.step(voltsForDegrees(10.0), PodAlignmentController.SETTLE_SECONDS);
        assertTrue(controller.isFailed());
        assertFalse(controller.isComplete());
    }

    @Test
    public void confirmedCalibrationIsReadyForPoweredCommissioning() {
        assertTrue(SwervePodEncoder.CALIBRATION_VERIFIED);
        assertTrue(SwervePodEncoder.calibrationReady());
        assertEquals(13.725, SwervePodEncoder.forwardTargetDegrees(true), EPSILON);
        assertEquals(29.025, SwervePodEncoder.forwardTargetDegrees(false), EPSILON);
        assertEquals(-1, SwervePodEncoder.LEFT_ANALOG_SIGN);
        assertEquals(-1, SwervePodEncoder.RIGHT_ANALOG_SIGN);
    }

    private static double voltsForDegrees(double degrees) {
        return degrees / 360.0 * 3.2;
    }

    private static double radiansForCounts(double counts) {
        return counts * 2.0 * Math.PI / SwervePodEncoder.COUNTS_PER_REVOLUTION;
    }
}
