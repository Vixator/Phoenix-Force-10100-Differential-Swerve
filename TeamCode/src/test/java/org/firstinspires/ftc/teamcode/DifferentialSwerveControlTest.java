package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class DifferentialSwerveControlTest {
    private static final double EPSILON = 1e-10;

    @Test
    public void stickDeadbandsStopNoiseAndRescaleWithoutChangingDirection() {
        SwerveDriverInput input = new SwerveDriverInput();
        input.update(0.03, -0.03, 0.05);
        assertEquals(0.0, input.getForward(), EPSILON);
        assertEquals(0.0, input.getStrafe(), EPSILON);
        assertEquals(0.0, input.getTurn(), EPSILON);
        input.update(0.315, -0.42, -0.525); // Radial magnitude .525 becomes .5; turn becomes -.5 cubed.
        assertEquals(0.4, input.getForward(), EPSILON);
        assertEquals(0.3, input.getStrafe(), EPSILON);
        assertEquals(-0.125 * SwerveDriverInput.MAX_TURN_RATE, input.getTurn(), EPSILON);
        input.update(1.0, -1.0, 1.0);
        assertEquals(1.0, Math.hypot(input.getForward(), input.getStrafe()), EPSILON);
        assertEquals(input.getForward(), input.getStrafe(), EPSILON);
        assertEquals(SwerveDriverInput.MAX_TURN_RATE, input.getTurn(), EPSILON);
        input.update(0.0, 0.0, 0.0);
        assertEquals(0.0, input.getForward(), EPSILON);
        assertEquals(0.0, input.getStrafe(), EPSILON);
        assertEquals(0.0, input.getTurn(), EPSILON);
    }

    @Test
    public void pureRotationUsesBothMotorsOnBothPodsWithCubicStickScaling() {
        for (double direction : new double[]{-1.0, 1.0}) {
            for (double stick : new double[]{0.525, 1.0}) {
                SwerveDriverInput input = new SwerveDriverInput();
                input.update(0.0, 0.0, direction * stick);
                DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
                kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
                DifferentialSwervePodController left = new DifferentialSwervePodController();
                DifferentialSwervePodController right = new DifferentialSwervePodController();
                update(left, 0.0, 0.0, kinematics.getLeftTargetAngle(), kinematics.getLeftSpeed(), 0.02);
                update(right, 0.0, 0.0, kinematics.getRightTargetAngle(), kinematics.getRightSpeed(), 0.02);
                double expected = direction * (stick == 1.0 ? 1.0 : 0.125);
                assertEquals(expected, left.getLeftMotorCommand(), EPSILON);
                assertEquals(expected, left.getRightMotorCommand(), EPSILON);
                assertEquals(-expected, right.getLeftMotorCommand(), EPSILON);
                assertEquals(-expected, right.getRightMotorCommand(), EPSILON);
                assertEquals(0.0, left.getSteeringCommand(), EPSILON);
                assertEquals(0.0, right.getSteeringCommand(), EPSILON);
            }
        }
    }

    @Test
    public void fullRotationWithTranslationNormalizesBothVectorsTogether() {
        SwerveDriverInput input = new SwerveDriverInput();
        input.update(0.0, -1.0, 1.0);
        DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
        kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
        assertEquals(1.0, kinematics.getLeftSpeed(), EPSILON);
        assertEquals(0.0, kinematics.getRightSpeed(), EPSILON);

        input.update(1.0, 0.0, 1.0);
        kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
        assertEquals(1.0, kinematics.getLeftSpeed(), EPSILON);
        assertEquals(1.0, kinematics.getRightSpeed(), EPSILON);
        assertEquals(Math.PI / 4.0, kinematics.getLeftTargetAngle(), EPSILON);
        assertEquals(3.0 * Math.PI / 4.0, kinematics.getRightTargetAngle(), EPSILON);
    }

    @Test
    public void fullRotationStillReservesMotorHeadroomForSteering() {
        SwerveDriverInput input = new SwerveDriverInput();
        input.update(0.0, 0.0, 1.0);
        DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
        kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
        DifferentialSwervePodController controller = new DifferentialSwervePodController();
        update(controller, Math.toRadians(10.0), 0.0,
                kinematics.getLeftTargetAngle(), kinematics.getLeftSpeed(), 0.02);
        assertTrue(controller.getSteeringCommand() > 0.0);
        assertTrue(Math.abs(controller.getLeftMotorCommand()) <= 1.0);
        assertTrue(Math.abs(controller.getRightMotorCommand()) <= 1.0);
        assertEquals(controller.getSteeringCommand(),
                (controller.getLeftMotorCommand() - controller.getRightMotorCommand()) / 2.0, EPSILON);
        assertTrue((controller.getLeftMotorCommand() + controller.getRightMotorCommand()) / 2.0 < 1.0);
    }

    @Test
    public void steeringDoesNotFlipBackAndForthAtNinetyDegrees() {
        for (double direction : new double[]{-1.0, 1.0}) {
            DifferentialSwervePodController controller = new DifferentialSwervePodController();
            for (double degrees : new double[]{89.0, 90.1, 89.9, 91.0}) {
                update(controller, 0.0, 0.0, direction * Math.toRadians(degrees), 0.5, 0.1);
                assertTrue(direction * controller.getAngleError() > 0.0);
                if (degrees > 90.0) {
                    assertEquals(0.0, (controller.getLeftMotorCommand() + controller.getRightMotorCommand()) / 2.0, EPSILON);
                }
            }
            update(controller, 0.0, 0.0, direction * Math.toRadians(94.0), 0.5, 0.1);
            assertTrue(direction * controller.getAngleError() < 0.0);
            update(controller, 0.0, 0.0, direction * Math.toRadians(89.0), 0.5, 0.1);
            assertTrue(direction * controller.getAngleError() < 0.0);
            update(controller, 0.0, 0.0, direction * Math.toRadians(86.0), 0.5, 0.1);
            assertTrue(direction * controller.getAngleError() > 0.0);
        }
    }

    @Test
    public void onePodCanStopAtTheInstantaneousCenterOfRotation() {
        DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
        kinematics.update(0.0, 0.5, 0.0);
        double turnSpeed = 2.1 * HardwareConstants.TRACK_WIDTH_METERS
                / (2.0 * HardwareConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND);
        kinematics.update(turnSpeed, 0.0, 2.1);
        assertEquals(0.0, kinematics.getRightSpeed(), EPSILON);
        assertEquals(Math.PI / 2.0, kinematics.getRightTargetAngle(), EPSILON);
        assertEquals(2.0 * turnSpeed, kinematics.getLeftSpeed(), EPSILON);
        assertEquals(0.0, kinematics.getLeftTargetAngle(), EPSILON);
    }

    @Test
    public void optimizedWheelVectorsMatchTranslationInEveryQuadrant() {
        for (double degrees : new double[]{-179.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, 180.0}) {
            double target = Math.toRadians(degrees);
            DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
            kinematics.update(0.5 * Math.cos(target), 0.5 * Math.sin(target), 0.0);
            DifferentialSwervePodController controller = new DifferentialSwervePodController();
            update(controller, 0.0, 0.0, kinematics.getLeftTargetAngle(), kinematics.getLeftSpeed(), 0.02);
            double alignedAngle = controller.getOptimizedTargetAngle();
            update(controller, alignedAngle, 0.0, kinematics.getLeftTargetAngle(), kinematics.getLeftSpeed(), 1.0);
            double drive = (controller.getLeftMotorCommand() + controller.getRightMotorCommand()) / 2.0;
            assertEquals(0.5 * Math.cos(target), drive * Math.cos(alignedAngle), EPSILON);
            assertEquals(0.5 * Math.sin(target), drive * Math.sin(alignedAngle), EPSILON);
            assertEquals(0.0, controller.getSteeringCommand(), EPSILON);
        }
    }

    @Test
    public void runtimeAndStartupBothCorrectTowardForwardWithMeasuredMotorPolarity() {
        double[] references = {SwervePodEncoder.LEFT_FORWARD_DEGREES, SwervePodEncoder.RIGHT_FORWARD_DEGREES};
        int[] analogSigns = {SwervePodEncoder.LEFT_ANALOG_SIGN, SwervePodEncoder.RIGHT_ANALOG_SIGN};
        for (int pod = 0; pod < references.length; pod++) {
            for (double angleDegrees : new double[]{-30.0, 30.0}) {
                double rawDegrees = (references[pod] + angleDegrees / analogSigns[pod] + 360.0) % 360.0;
                double volts = rawDegrees * SwervePodEncoder.FULL_SCALE_VOLTS / 360.0;
                double angle = SwervePodEncoder.absoluteRadians(volts, references[pod], analogSigns[pod]);
                PodAlignmentController startup = new PodAlignmentController(references[pod], analogSigns[pod]);
                startup.start();
                startup.step(volts, 0.02);
                DifferentialSwervePodController runtime = new DifferentialSwervePodController();
                update(runtime, angle, 0.0, 0.0, 0.0, 0.02);
                assertEquals(Math.signum(startup.getCommand()), Math.signum(runtime.getSteeringCommand()), EPSILON);
                // Physical motor response: positive left and negative right rotate CCW.
                double clockwiseMotion = clockwiseMotionFromMotors(runtime);
                assertTrue("Correction must reduce forward error", angle * clockwiseMotion < 0.0);
            }
        }
    }

    @Test
    public void measuredRateDampingOpposesMotionInBothDirections() {
        for (double rate : new double[]{-2.0, 2.0}) {
            DifferentialSwervePodController controller = new DifferentialSwervePodController();
            update(controller, 0.0, rate, 0.0, 0.0, 0.02);
            assertTrue("D must brake, not accelerate the pod", rate * clockwiseMotionFromMotors(controller) < 0.0);
        }
    }

    @Test
    public void smallStrafeRequestsSteerTowardTheRequestedSide() {
        for (double strafe : new double[]{-0.01, 0.01}) {
            DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
            kinematics.update(0.0, strafe, 0.0);
            DifferentialSwervePodController controller = new DifferentialSwervePodController();
            update(controller, 0.0, 0.0, kinematics.getLeftTargetAngle(), kinematics.getLeftSpeed(), 0.02);
            assertTrue(strafe * clockwiseMotionFromMotors(controller) > 0.0);
            // At 90 degrees the wheel must align before propelling the robot.
            assertEquals(0.0, (controller.getLeftMotorCommand() + controller.getRightMotorCommand()) / 2.0, EPSILON);
        }
    }

    @Test
    public void pureChassisTurnDrivesOppositeWheelsWithoutRotatingThePods() {
        for (double turn : new double[]{-2.1, 2.1}) {
            DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
            kinematics.update(0.0, 0.0, turn);
            DifferentialSwervePodController left = new DifferentialSwervePodController();
            DifferentialSwervePodController right = new DifferentialSwervePodController();
            update(left, 0.0, 0.0, kinematics.getLeftTargetAngle(), kinematics.getLeftSpeed(), 0.02);
            update(right, 0.0, 0.0, kinematics.getRightTargetAngle(), kinematics.getRightSpeed(), 0.02);
            assertEquals(0.0, left.getSteeringCommand(), EPSILON);
            assertEquals(0.0, right.getSteeringCommand(), EPSILON);
            assertTrue(turn * left.getLeftMotorCommand() > 0.0);
            assertTrue(turn * right.getLeftMotorCommand() < 0.0);
            assertEquals(-left.getLeftMotorCommand(), right.getLeftMotorCommand(), EPSILON);
        }
    }

    @Test
    public void reverseTravelUsesWheelReversalInsteadOfHalfTurn() {
        DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
        kinematics.update(-0.5, 0.0, 0.0);
        DifferentialSwervePodController controller = new DifferentialSwervePodController();
        update(controller, 0.0, 0.0, kinematics.getLeftTargetAngle(), kinematics.getLeftSpeed(), 0.02);
        assertEquals(0.0, controller.getAngleError(), EPSILON);
        assertEquals(-0.5, controller.getLeftMotorCommand(), EPSILON);
        assertEquals(-0.5, controller.getRightMotorCommand(), EPSILON);
    }

    @Test
    public void wrapBoundaryUsesShortCorrection() {
        DifferentialSwervePodController controller = new DifferentialSwervePodController();
        update(controller, Math.toRadians(179.0), 0.0, Math.toRadians(-179.0), 0.2, 0.02);
        assertEquals(Math.toRadians(2.0), controller.getAngleError(), EPSILON);
        assertTrue(clockwiseMotionFromMotors(controller) > 0.0);
    }

    @Test
    public void stoppedVectorsRetainTargetsAndCombinedRequestsPreserveTheirRatio() {
        DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
        kinematics.update(1.0, 1.0, 2.1);
        assertTrue(kinematics.getLeftSpeed() <= 1.0);
        assertTrue(kinematics.getRightSpeed() <= 1.0);
        double turnSpeed = 2.1 * HardwareConstants.TRACK_WIDTH_METERS
                / (2.0 * HardwareConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND);
        assertEquals(Math.hypot(1.0 + turnSpeed, 1.0) / Math.hypot(1.0 - turnSpeed, 1.0),
                kinematics.getLeftSpeed() / kinematics.getRightSpeed(), EPSILON);
        double leftTarget = kinematics.getLeftTargetAngle();
        double rightTarget = kinematics.getRightTargetAngle();
        kinematics.update(0.0, 0.0, 0.0);
        assertEquals(leftTarget, kinematics.getLeftTargetAngle(), EPSILON);
        assertEquals(rightTarget, kinematics.getRightTargetAngle(), EPSILON);
        assertEquals(0.0, kinematics.getLeftSpeed(), EPSILON);
        assertEquals(0.0, kinematics.getRightSpeed(), EPSILON);
    }

    @Test
    public void reducedSteeringLimitOverridesPreviousSlewStateAndMotorsRemainBounded() {
        DifferentialSwervePodController controller = new DifferentialSwervePodController();
        controller.update(0.0, 0.0, Math.PI / 4.0, 1.0, 0.1, 4.0, 0.0, 1.0, 1.0, 0.0);
        controller.update(0.0, 0.0, Math.PI / 4.0, 1.0, 0.01, 4.0, 0.0, 1.0, 0.1, 2.0);
        assertEquals(-0.1, controller.getSteeringCommand(), EPSILON);
        assertTrue(Math.abs(controller.getLeftMotorCommand()) <= 1.0);
        assertTrue(Math.abs(controller.getRightMotorCommand()) <= 1.0);
        assertEquals(controller.getSteeringCommand(),
                (controller.getLeftMotorCommand() - controller.getRightMotorCommand()) / 2.0, EPSILON);
    }

    @Test
    public void closedLoopConvergesAndHoldsAfterStickReleaseWithDocumentedEncoderPolarity() {
        // A simple first-order motor model exercises the production controller + tracker.
        // It catches feedback inversion; it does not establish real-robot tuning.
        for (int sign : new int[]{SwervePodEncoder.LEFT_QUADRATURE_SIGN, SwervePodEncoder.RIGHT_QUADRATURE_SIGN}) {
            for (double seconds : new double[]{0.01, 0.02, 0.05}) {
                for (double target : new double[]{-90.0, -45.0, 45.0, 90.0, 170.0}) {
                    DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
                    kinematics.update(0.1 * Math.cos(Math.toRadians(target)), 0.1 * Math.sin(Math.toRadians(target)), 0.0);
                    SwervePodEncoder encoder = new SwervePodEncoder(sign);
                    encoder.seed(0.0, 0);
                    DifferentialSwervePodController controller = new DifferentialSwervePodController();
                    double angle = 0.0;
                    double rate = 0.0;
                    double maxRate = HardwareConstants.MOTOR_FREE_SPEED_RPM / 60.0
                            * HardwareConstants.FIRST_STAGE_RATIO * 2.0 * Math.PI;
                    for (int step = 0; step < (int) (5.0 / seconds); step++) {
                        if (step * seconds >= 2.5) kinematics.update(0.0, 0.0, 0.0);
                        update(controller, encoder.getAngleRadians(), encoder.getRateRadiansPerSecond(),
                                kinematics.getLeftTargetAngle(), kinematics.getLeftSpeed(), seconds);
                        rate += (clockwiseMotionFromMotors(controller) * maxRate - rate)
                                * (1.0 - Math.exp(-seconds / 0.04));
                        angle += rate * seconds;
                        // CW motion increases RAW counts on the measured hardware.
                        encoder.update((int) Math.round(angle * SwervePodEncoder.COUNTS_PER_REVOLUTION / (2.0 * Math.PI)), seconds);
                        assertTrue(Math.abs(controller.getLeftMotorCommand()) <= 1.0);
                        assertTrue(Math.abs(controller.getRightMotorCommand()) <= 1.0);
                    }
                    double error = SwervePodEncoder.wrapRadians(Math.toRadians(target) - angle);
                    if (Math.abs(error) > Math.PI / 2.0) error -= Math.copySign(Math.PI, error);
                    assertEquals("Pod must converge rather than run away", 0.0, error, Math.toRadians(1.0));
                    assertEquals("Released sticks must stop wheel drive", 0.0,
                            (controller.getLeftMotorCommand() + controller.getRightMotorCommand()) / 2.0, EPSILON);
                }
            }
        }
    }

    @Test
    public void cancelledRightVectorResumesTranslationWithoutCounterclockwiseTurn() {
        for (double startingAngle : new double[]{-Math.PI, -Math.PI / 2.0, 0.0, Math.PI / 2.0}) {
            SwerveDriverInput input = new SwerveDriverInput();
            DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
            DifferentialSwervePodController right = new DifferentialSwervePodController();
            // Establish a previous travel axis, then cancel the right vector with
            // equal forward and clockwise requests for long enough to settle slew.
            kinematics.update(Math.cos(startingAngle), Math.sin(startingAngle), 0.0);
            update(right, startingAngle, 0.0, kinematics.getRightTargetAngle(), kinematics.getRightSpeed(), 0.02);
            input.update(0.0, -1.0, 1.0);
            kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
            for (int step = 0; step < 100; step++) {
                update(right, startingAngle, 0.0, kinematics.getRightTargetAngle(), kinematics.getRightSpeed(), 0.02);
            }
            assertEquals(0.0, right.getLeftMotorCommand(), EPSILON);
            assertEquals(0.0, right.getRightMotorCommand(), EPSILON);

            // Center rotation and request either forward or sideways translation.
            for (double[] stick : new double[][]{{0.0, -1.0}, {1.0, 0.0}}) {
                input.update(stick[0], stick[1], 0.0);
                kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
                for (int step = 0; step < 100; step++) {
                    update(right, startingAngle, 0.0, kinematics.getRightTargetAngle(), kinematics.getRightSpeed(), 0.02);
                }
                assertTrue("Translation must request drive or steering even with a stationary pod",
                        Math.max(Math.abs(right.getLeftMotorCommand()), Math.abs(right.getRightMotorCommand())) > 0.1);
            }
        }
    }

    @Test
    public void alignmentSuppressionCannotLatchBothMotorsOffWithLargeSteeringError() {
        for (double direction : new double[]{-1.0, 1.0}) {
            DifferentialSwervePodController controller = new DifferentialSwervePodController();
            // Retain the unreversed choice just beyond 90 degrees, where wheel
            // drive is deliberately suppressed. Steering must continue indefinitely.
            update(controller, 0.0, 0.0, direction * Math.toRadians(89.0), 1.0, 0.02);
            for (int step = 0; step < 500; step++) {
                update(controller, 0.0, 0.0, direction * Math.toRadians(92.0), 1.0, 0.02);
                assertEquals(0.0,
                        (controller.getLeftMotorCommand() + controller.getRightMotorCommand()) / 2.0, EPSILON);
                assertTrue("A stationary misaligned pod must keep receiving a steering correction",
                        direction * clockwiseMotionFromMotors(controller) > 0.0);
            }
        }
    }

    @Test
    public void partialStickCancellationStopsRightPodAndEitherTurnChangeRestoresCommands() {
        for (double travelSign : new double[]{-1.0, 1.0}) {
            for (double nextTurn : new double[]{0.3, 0.7}) {
                SwerveDriverInput input = new SwerveDriverInput();
                DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
                DifferentialSwervePodController right = new DifferentialSwervePodController();
                // Translation: (.16875 - .05) / .95 = .125.
                // Rotation: ((.525 - .05) / .95)^3 = .125.
                input.update(0.0, -travelSign * 0.16875, travelSign * 0.525);
                kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
                update(right, 0.0, 0.0, kinematics.getRightTargetAngle(), kinematics.getRightSpeed(), 0.02);
                assertEquals(0.25, kinematics.getLeftSpeed(), EPSILON);
                assertEquals(0.0, kinematics.getRightSpeed(), EPSILON);
                assertEquals(0.0, right.getLeftMotorCommand(), EPSILON);
                assertEquals(0.0, right.getRightMotorCommand(), EPSILON);

                // Keeping translation unchanged, either increasing or decreasing
                // the same-direction turn request removes the cancellation.
                input.update(0.0, -travelSign * 0.16875, travelSign * nextTurn);
                kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
                update(right, 0.0, 0.0, kinematics.getRightTargetAngle(), kinematics.getRightSpeed(), 0.02);
                assertTrue(Math.abs(right.getLeftMotorCommand()) > 0.1);
                assertTrue(Math.abs(right.getRightMotorCommand()) > 0.1);
            }
        }
    }

    @Test
    public void changingTurnCanRestoreDriveWhileMisalignedPodRemainsPhysicallyStationary() {
        SwerveDriverInput input = new SwerveDriverInput();
        DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
        DifferentialSwervePodController right = new DifferentialSwervePodController();
        input.update(0.3, 0.0, 0.0);
        kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
        for (int step = 0; step < 100; step++) {
            // Deliberately hold measured angle/rate stationary to represent a pod
            // unable to execute steering. This does not model or prove a real jam.
            update(right, 0.0, 0.0, kinematics.getRightTargetAngle(), kinematics.getRightSpeed(), 0.02);
        }
        assertEquals(0.0, (right.getLeftMotorCommand() + right.getRightMotorCommand()) / 2.0, EPSILON);
        assertEquals(-DifferentialSwervePodController.DEFAULT_MAX_STEER, right.getSteeringCommand(), EPSILON);

        input.update(0.3, 0.0, -1.0);
        kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
        for (int step = 0; step < 100; step++) {
            update(right, 0.0, 0.0, kinematics.getRightTargetAngle(), kinematics.getRightSpeed(), 0.02);
        }
        assertTrue("A turn changes the desired axis enough to restore wheel drive without any encoder movement",
                (right.getLeftMotorCommand() + right.getRightMotorCommand()) / 2.0 > 0.8);
    }

    @Test
    public void mixedInputHistoryDoesNotPreventEitherPodFromSettlingOnTranslation() {
        Random random = new Random(10100L);
        double maxRate = HardwareConstants.MOTOR_FREE_SPEED_RPM / 60.0
                * HardwareConstants.FIRST_STAGE_RATIO * 2.0 * Math.PI;
        for (boolean rightPod : new boolean[]{false, true}) {
            for (double seconds : new double[]{0.01, 0.02, 0.04}) {
                for (int trial = 0; trial < 40; trial++) {
                    SwerveDriverInput input = new SwerveDriverInput();
                    DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
                    DifferentialSwervePodController controller = new DifferentialSwervePodController();
                    SwervePodEncoder encoder = new SwervePodEncoder(rightPod
                            ? SwervePodEncoder.RIGHT_QUADRATURE_SIGN : SwervePodEncoder.LEFT_QUADRATURE_SIGN);
                    double angle = random.nextDouble() * 2.0 * Math.PI - Math.PI;
                    double rate = 0.0;
                    encoder.seed(angle, (int) Math.round(angle * SwervePodEncoder.COUNTS_PER_REVOLUTION / (2.0 * Math.PI)));
                    double finalTarget = random.nextDouble() * 2.0 * Math.PI - Math.PI;
                    int mixedSteps = (int) (3.0 / seconds);
                    int settlingSteps = (int) (6.0 / seconds);
                    for (int step = 0; step < mixedSteps + settlingSteps; step++) {
                        if (step < mixedSteps && step % 5 == 0) {
                            input.update(2.0 * random.nextDouble() - 1.0,
                                    2.0 * random.nextDouble() - 1.0, 2.0 * random.nextDouble() - 1.0);
                        } else if (step == mixedSteps) {
                            input.update(0.5 * Math.sin(finalTarget), -0.5 * Math.cos(finalTarget), 0.0);
                        }
                        kinematics.update(input.getForward(), input.getStrafe(), input.getTurn());
                        update(controller, encoder.getAngleRadians(), encoder.getRateRadiansPerSecond(),
                                rightPod ? kinematics.getRightTargetAngle() : kinematics.getLeftTargetAngle(),
                                rightPod ? kinematics.getRightSpeed() : kinematics.getLeftSpeed(), seconds);
                        // Include the measured right-pod hardware mapping in the
                        // physical model; it must preserve differential steering.
                        double motorLeft = rightPod ? -controller.getRightMotorCommand() : controller.getLeftMotorCommand();
                        double motorRight = rightPod ? -controller.getLeftMotorCommand() : controller.getRightMotorCommand();
                        assertTrue(Math.abs(motorLeft) <= 1.0);
                        assertTrue(Math.abs(motorRight) <= 1.0);
                        rate += ((motorRight - motorLeft) / 2.0 * maxRate - rate)
                                * (1.0 - Math.exp(-seconds / 0.04));
                        angle += rate * seconds;
                        encoder.update((int) Math.round(angle * SwervePodEncoder.COUNTS_PER_REVOLUTION / (2.0 * Math.PI)), seconds);
                    }
                    double drive = (controller.getLeftMotorCommand() + controller.getRightMotorCommand()) / 2.0;
                    assertEquals("Mixed input history must not latch translation off",
                            Math.hypot(input.getForward(), input.getStrafe()), Math.abs(drive), 0.002);
                    assertEquals(input.getForward(), drive * Math.cos(angle), 0.01);
                    assertEquals(input.getStrafe(), drive * Math.sin(angle), 0.01);
                }
            }
        }
    }

    private static double clockwiseMotionFromMotors(DifferentialSwervePodController controller) {
        // Independent physical model: left-positive is CCW, right-positive is CW.
        return (controller.getRightMotorCommand() - controller.getLeftMotorCommand()) / 2.0;
    }

    private static void update(DifferentialSwervePodController controller, double angle, double rate,
                               double target, double speed, double seconds) {
        controller.update(angle, rate, target, speed, seconds,
                DifferentialSwervePodController.DEFAULT_KP, DifferentialSwervePodController.DEFAULT_KD,
                1.0, DifferentialSwervePodController.DEFAULT_MAX_STEER, DifferentialSwervePodController.DEFAULT_SLEW_RATE);
    }
}
