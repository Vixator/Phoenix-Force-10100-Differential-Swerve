package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.SwervePod;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.LinkedHashMap;
import java.util.Map;

/** A Pedro {@link SwervePod} backed by one of the robot's differential pods. */
public final class DifferentialPod implements SwervePod {
    public interface OutputGate {
        void requireOutputAllowed();
        default void requireAlignmentOutputAllowed() {
            throw new IllegalStateException("Alignment is not authorized");
        }
        void latchFault(String message);
        boolean allowMaintenanceFloat();
        double snapshotAgeMillis();
        String faultDescription();
    }

    interface MotorIO {
        void setVelocity(double ticksPerSecond);
        double getVelocity();
        int getPosition();
        void setBrake();
        void setFloat();
    }

    interface CountSource { int get(); }
    interface VoltageSource { double get(); }

    private final String podName;
    private final boolean rightPod;
    private final Vector2D offset;
    private final MotorIO firstMotor;
    private final MotorIO secondMotor;
    private final CountSource quadrature;
    private final VoltageSource absolute;
    private final double forwardDegrees;
    private final int analogSign;
    private final double maxDrive;
    private final double maxMotorCommand;
    private final OutputGate gate;
    private final SwervePodEncoder encoder;
    private final DifferentialSwervePodController controller = new DifferentialSwervePodController();
    private final PodAlignmentController alignment;

    private double sampleSeconds = 0.02;
    private double requestedWheelAngle;
    private double requestedDrivePower;
    private double logicalLeft;
    private double logicalRight;
    private double firstTargetTicks;
    private double secondTargetTicks;
    private boolean prepared;
    private boolean alignmentPrepared;
    private boolean saturated;

    public static DifferentialPod forHardware(
            String name, boolean rightPod, DcMotorEx firstMotor, DcMotorEx secondMotor,
            DcMotor quadrature, AnalogInput absolute, double maxDrive, double maxMotorCommand, OutputGate gate) {
        return new DifferentialPod(name, rightPod, new HardwareMotor(firstMotor), new HardwareMotor(secondMotor),
                () -> quadrature.getController().getMotorCurrentPosition(quadrature.getPortNumber()),
                absolute::getVoltage, maxDrive, maxMotorCommand, gate);
    }

    DifferentialPod(String name, boolean rightPod, MotorIO firstMotor, MotorIO secondMotor,
                    CountSource quadrature, VoltageSource absolute, double maxDrive, double maxMotorCommand, OutputGate gate) {
        PedroDriveConfig.finiteRange("maxDrive", maxDrive, 0.0, 1.0);
        this.podName = name;
        this.rightPod = rightPod;
        this.firstMotor = firstMotor;
        this.secondMotor = secondMotor;
        this.quadrature = quadrature;
        this.absolute = absolute;
        PedroDriveConfig.finiteRange("maxMotorCommand", maxMotorCommand, maxDrive, 1.0);
        this.maxDrive = maxDrive;
        this.maxMotorCommand = maxMotorCommand;
        this.gate = gate;
        this.offset = Vector2D.cartesian(0.0,
                rightPod ? -HardwareConstants.POD_CENTER_OFFSET_INCHES
                        : HardwareConstants.POD_CENTER_OFFSET_INCHES);
        this.forwardDegrees = rightPod
                ? SwervePodEncoder.RIGHT_FORWARD_DEGREES : SwervePodEncoder.LEFT_FORWARD_DEGREES;
        this.analogSign = rightPod ? SwervePodEncoder.RIGHT_ANALOG_SIGN : SwervePodEncoder.LEFT_ANALOG_SIGN;
        this.encoder = new SwervePodEncoder(rightPod
                ? SwervePodEncoder.RIGHT_QUADRATURE_SIGN : SwervePodEncoder.LEFT_QUADRATURE_SIGN);
        this.alignment = new PodAlignmentController(forwardDegrees, analogSign);
    }

    @Override public String name() { return podName; }
    @Override public Vector2D getOffset() { return offset; }
    @Override public double getAngle() { return encoder.getAngleRadians(); }
    @Override public double adjustThetaForEncoder(double wheelTheta) {
        return PedroSwerveMath.encoderAngle(wheelTheta);
    }

    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        try {
            prepareMove(targetAngleRad, drivePower, ignoreAngleChanges);
            commitPrepared();
        } catch (RuntimeException exception) {
            stopAfterFailure(exception);
            throw exception;
        }
    }

    void prepareMove(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        prepared = false;
        alignmentPrepared = false;
        gate.requireOutputAllowed();
        if (!Double.isFinite(targetAngleRad) || !Double.isFinite(drivePower)) {
            throw new IllegalArgumentException(podName + " received a nonfinite command");
        }
        drivePower = PedroSwerveMath.clamp(drivePower, -1.0, 1.0);
        requestedWheelAngle = SwervePodEncoder.wrapRadians(targetAngleRad);
        requestedDrivePower = drivePower;
        double encoderTarget = adjustThetaForEncoder(requestedWheelAngle);

        if (ignoreAngleChanges && Math.abs(drivePower) <= 1e-12) {
            prepareZero();
            return;
        }
        if (ignoreAngleChanges) controller.stopOutput();
        SwerveTuning.validate();
        if (maxDrive + SwerveTuning.STEERING_MAX_COMMAND > maxMotorCommand + 1e-12
                && maxMotorCommand < 1.0) {
            throw new IllegalArgumentException("Live steering plus drive exceeds the commissioning motor cap");
        }
        controller.update(encoder.getAngleRadians(), encoder.getRateRadiansPerSecond(),
                encoderTarget, drivePower, sampleSeconds,
                SwerveTuning.STEERING_P, ignoreAngleChanges ? 0.0 : SwerveTuning.STEERING_D,
                maxDrive, ignoreAngleChanges ? 0.0 : SwerveTuning.STEERING_MAX_COMMAND,
                ignoreAngleChanges ? 0.0 : SwerveTuning.STEERING_SLEW_RATE);
        prepareLogical(controller.getLeftMotorCommand(), controller.getRightMotorCommand());
    }

    void prepareZero() {
        controller.stopOutput();
        prepareLogical(0.0, 0.0);
    }

    private void prepareLogical(double left, double right) {
        if (!Double.isFinite(left) || !Double.isFinite(right)
                || Math.max(Math.abs(left), Math.abs(right)) > maxMotorCommand + 1e-12) {
            throw new IllegalArgumentException(podName + " motor command exceeds its output cap");
        }
        logicalLeft = left;
        logicalRight = right;
        double[] targets = hardwareTargets(rightPod, left, right);
        firstTargetTicks = targets[0];
        secondTargetTicks = targets[1];
        saturated = Math.max(Math.abs(left), Math.abs(right)) >= maxMotorCommand - 1e-12;
        prepared = true;
    }

    void commitPrepared() {
        if (!prepared) throw new IllegalStateException(podName + " has no prepared output");
        try {
            if (alignmentPrepared) gate.requireAlignmentOutputAllowed(); else gate.requireOutputAllowed();
            firstMotor.setVelocity(firstTargetTicks);
            if (alignmentPrepared) gate.requireAlignmentOutputAllowed(); else gate.requireOutputAllowed();
            secondMotor.setVelocity(secondTargetTicks);
        } catch (RuntimeException exception) {
            stopAfterFailure(exception);
            throw exception;
        } finally {
            prepared = false;
        }
    }

    private void stopAfterFailure(RuntimeException exception) {
        try { safeZero(); } catch (RuntimeException stopFailure) { exception.addSuppressed(stopFailure); }
        gate.latchFault(podName + " output failed: " + exception.getMessage());
    }

    static double[] hardwareTargets(boolean rightPod, double logicalLeft, double logicalRight) {
        double scale = HardwareConstants.MAX_MOTOR_TICKS_PER_SECOND;
        return rightPod
                ? new double[] {-logicalRight * scale, -logicalLeft * scale}
                : new double[] {logicalLeft * scale, logicalRight * scale};
    }

    void acceptSample(double seconds) {
        if (!Double.isFinite(seconds) || seconds <= 0.0) {
            throw new HubSnapshotReader.FeedbackFault(podName + " has invalid sample time");
        }
        sampleSeconds = seconds;
        encoder.update(quadrature.get(), seconds);
    }

    void seedAtStart() {
        double volts = absolute.get();
        if (!SwervePodEncoder.validVoltage(volts)) {
            throw new IllegalStateException(podName + ": " + SwervePodEncoder.voltageFault(volts));
        }
        double angle = SwervePodEncoder.absoluteRadians(volts, forwardDegrees, analogSign);
        if (Math.abs(Math.toDegrees(angle)) > PodAlignmentController.TOLERANCE_DEGREES) {
            throw new IllegalStateException(podName + " moved after INIT alignment: "
                    + Math.toDegrees(angle) + " deg; reinitialize to realign");
        }
        encoder.seed(angle, quadrature.get());
        controller.stopOutput();
    }

    void verifyRecoveredAngle() {
        double volts = absolute.get();
        if (!SwervePodEncoder.validVoltage(volts)) {
            throw new HubSnapshotReader.FeedbackFault(podName + ": " + SwervePodEncoder.voltageFault(volts));
        }
        double difference = SwervePodEncoder.wrapRadians(encoder.getAngleRadians()
                - SwervePodEncoder.absoluteRadians(volts, forwardDegrees, analogSign));
        if (Math.abs(difference) > Math.toRadians(10.0)) {
            throw new HubSnapshotReader.FeedbackFault(podName
                    + " feedback disagrees after hub recovery by " + Math.toDegrees(difference) + " deg");
        }
    }

    void startAlignment() { alignment.start(); }

    void prepareAlignment(double seconds) {
        prepared = false;
        alignmentPrepared = true;
        gate.requireAlignmentOutputAllowed();
        double volts = absolute.get();
        if (!SwervePodEncoder.validVoltage(volts)) {
            alignment.abort(SwervePodEncoder.voltageFault(volts));
        } else if (alignment.isComplete() && Math.abs(SwervePodEncoder.analogErrorDegrees(
                volts, alignment.getTargetDegrees(), analogSign)) > PodAlignmentController.TOLERANCE_DEGREES) {
            alignment.abort("moved away from forward after settling");
        }
        if (SwerveTuning.ALIGNMENT_MAX_COMMAND > maxMotorCommand) {
            throw new IllegalArgumentException("Live alignment command exceeds the motor cap");
        }
        alignment.step(volts, seconds);
        prepareLogical(alignment.getCommand(), -alignment.getCommand());
    }

    boolean alignmentComplete() { return alignment.isComplete(); }
    boolean alignmentFailed() { return alignment.isFailed(); }
    String alignmentStatus() { return alignment.getStatus(); }
    double alignmentErrorDegrees() { return alignment.getErrorDegrees(); }
    double absoluteVoltage() { return absolute.get(); }
    int quadratureCount() { return encoder.getCount(); }

    @Override
    public void setToFloat() {
        RuntimeException failure = null;
        try { safeZero(); } catch (RuntimeException exception) { failure = exception; }
        if (!gate.allowMaintenanceFloat()) {
            IllegalStateException rejected = new IllegalStateException(
                    "FLOAT is allowed only in an explicitly disabled maintenance state");
            if (failure != null) rejected.addSuppressed(failure);
            gate.latchFault(podName + " rejected an unauthorized FLOAT request");
            throw rejected;
        }
        if (failure != null) throw failure; // Never FLOAT a motor whose zero command failed.
        Cleanup.runAll(firstMotor::setFloat, secondMotor::setFloat);
    }

    @Override public void setToBreak() {
        Cleanup.runAll(firstMotor::setBrake, secondMotor::setBrake);
    }

    public void safeZero() {
        controller.stopOutput();
        logicalLeft = logicalRight = firstTargetTicks = secondTargetTicks = 0.0;
        requestedDrivePower = 0.0;
        saturated = false;
        prepared = false;
        Cleanup.runAll(() -> firstMotor.setVelocity(0.0), () -> secondMotor.setVelocity(0.0));
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new LinkedHashMap<>();
        double volts = absolute.get();
        double absoluteAngle = SwervePodEncoder.validVoltage(volts)
                ? SwervePodEncoder.absoluteRadians(volts, forwardDegrees, analogSign) : Double.NaN;
        double averageMotorTicks = (firstMotor.getVelocity() + secondMotor.getVelocity()) / 2.0;
        if (rightPod) averageMotorTicks = -averageMotorTicks;
        map.put("targetWheelRad", requestedWheelAngle);
        map.put("actualEncoderRad", encoder.getAngleRadians());
        map.put("optimizedTargetRad", controller.getOptimizedTargetAngle());
        map.put("errorRad", controller.getAngleError());
        map.put("rateRadPerSec", encoder.getRateRadiansPerSecond());
        map.put("rawCount", encoder.getCount());
        map.put("absoluteVolts", volts);
        map.put("absoluteAngleRad", absoluteAngle);
        map.put("quadratureMinusAbsoluteRad",
                SwervePodEncoder.wrapRadians(encoder.getAngleRadians() - absoluteAngle));
        map.put("reversed", controller.isReversed());
        map.put("requestedDrive", requestedDrivePower);
        map.put("driveCommand", (logicalLeft + logicalRight) / 2.0);
        map.put("steeringCommand", controller.getSteeringCommand());
        map.put("logicalLeft", logicalLeft);
        map.put("logicalRight", logicalRight);
        map.put("firstTargetTicks", firstTargetTicks);
        map.put("secondTargetTicks", secondTargetTicks);
        map.put("firstMeasuredTicks", firstMotor.getVelocity());
        map.put("secondMeasuredTicks", secondMotor.getVelocity());
        map.put("firstCount", firstMotor.getPosition());
        map.put("secondCount", secondMotor.getPosition());
        map.put("moduleSpeedMetersPerSecond", averageMotorTicks
                / HardwareConstants.MOTOR_TICKS_PER_REVOLUTION * HardwareConstants.TOTAL_DRIVE_RATIO
                * HardwareConstants.WHEEL_CIRCUMFERENCE_METERS);
        map.put("saturated", saturated);
        map.put("snapshotAgeMs", gate.snapshotAgeMillis());
        map.put("fault", gate.faultDescription());
        return map;
    }

    private static final class HardwareMotor implements MotorIO {
        private final DcMotorEx motor;
        HardwareMotor(DcMotorEx motor) { this.motor = motor; }
        @Override public void setVelocity(double ticksPerSecond) { motor.setVelocity(ticksPerSecond); }
        @Override public double getVelocity() { return motor.getVelocity(); }
        @Override public int getPosition() { return motor.getCurrentPosition(); }
        @Override public void setBrake() { motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
        @Override public void setFloat() { motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); }
    }
}
