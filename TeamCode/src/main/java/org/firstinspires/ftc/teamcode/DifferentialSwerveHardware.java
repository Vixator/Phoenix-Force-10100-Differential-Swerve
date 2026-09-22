package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.ArrayList;
import java.util.List;

/** FTC SDK boundary. Runtime state and fault tests use the same lifecycle with a fake host. */
final class DifferentialSwerveHardware implements DifferentialSwerveRuntime.Host {
    private static final String TAG = "DifferentialSwerve";
    private final LinearOpMode opMode;
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final SwerveHubSession hubs;
    private DifferentialPod leftPod;
    private DifferentialPod rightPod;
    private double appliedP = Double.NaN, appliedI = Double.NaN, appliedD = Double.NaN, appliedF = Double.NaN;

    DifferentialSwerveHardware(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hubs = new SwerveHubSession(opMode.hardwareMap, opMode::isStopRequested);
    }

    @Override public DifferentialPod[] initialize(DifferentialPod.OutputGate gate, double maxDrive,
                                                  double maxMotorCommand) {
        initializeHardware(gate, maxDrive, maxMotorCommand);
        hubs.initialize();
        return new DifferentialPod[] {leftPod, rightPod};
    }

    private void initializeHardware(DifferentialPod.OutputGate gate, double maxDrive, double maxMotorCommand) {
        String[] names = {HardwareConstants.MOTOR_LEFT_POD_LEFT, HardwareConstants.MOTOR_LEFT_POD_RIGHT,
                HardwareConstants.MOTOR_RIGHT_POD_LEFT, HardwareConstants.MOTOR_RIGHT_POD_RIGHT};
        for (int i = 0; i < motors.length; i++) {
            motors[i] = opMode.hardwareMap.get(DcMotorEx.class, names[i]);
            motors[i].setVelocity(0.0);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (motors[i].getPortNumber() != i || motors[i].getController() != motors[0].getController()) {
                throw new IllegalArgumentException("Configure motor0..motor3 on Control Hub ports 0..3");
            }
        }
        DcMotor leftQuadrature = opMode.hardwareMap.get(DcMotor.class, HardwareConstants.ENCODER_LEFT);
        DcMotor rightQuadrature = opMode.hardwareMap.get(DcMotor.class, HardwareConstants.ENCODER_RIGHT);
        if (leftQuadrature.getPortNumber() != 0 || rightQuadrature.getPortNumber() != 1
                || leftQuadrature.getController() != rightQuadrature.getController()
                || leftQuadrature.getController() == motors[0].getController()) {
            throw new IllegalArgumentException(
                    "Configure encoderleft/encoderright on separate Expansion Hub ports 0/1");
        }
        AnalogInput leftAbsolute = opMode.hardwareMap.get(AnalogInput.class, HardwareConstants.ANALOG_LEFT);
        AnalogInput rightAbsolute = opMode.hardwareMap.get(AnalogInput.class, HardwareConstants.ANALOG_RIGHT);
        leftPod = DifferentialPod.forHardware("leftPod", false, motors[0], motors[1],
                leftQuadrature, leftAbsolute, maxDrive, maxMotorCommand, gate);
        rightPod = DifferentialPod.forHardware("rightPod", true, motors[2], motors[3],
                rightQuadrature, rightAbsolute, maxDrive, maxMotorCommand, gate);
        leftPod.setToBreak();
        rightPod.setToBreak();
        applyMotorPidfIfChanged();
    }

    @Override public String refreshHubs() { return hubs.refresh(); }

    public void applyMotorPidfIfChanged() {
        SwerveTuning.validate();
        double p = SwerveTuning.MOTOR_VELOCITY_P;
        double i = SwerveTuning.MOTOR_VELOCITY_I;
        double d = SwerveTuning.MOTOR_VELOCITY_D;
        double f = SwerveTuning.MOTOR_VELOCITY_F;
        if (p == appliedP && i == appliedI && d == appliedD && f == appliedF) return;
        for (DcMotorEx motor : motors) {
            if (motor != null) motor.setVelocityPIDFCoefficients(p, i, d, f);
        }
        appliedP = p;
        appliedI = i;
        appliedD = d;
        appliedF = f;
    }


    @Override public boolean stopRequested() { return opMode.isStopRequested(); }
    @Override public boolean started() { return opMode.isStarted(); }
    @Override public void pause() { opMode.sleep(HubSnapshotReader.RETRY_DELAY_MS); }
    @Override public void idle() { opMode.idle(); }
    @Override public void log(String message) { RobotLog.ee(TAG, message); }
    @Override public void showHealth(int recovered, String failure) {
        opMode.telemetry.addData("Recovered hub reads", recovered);
        if (recovered > 0) opMode.telemetry.addData("Last hub read failure", failure);
    }
    @Override public void showAlignment(DifferentialPod left, DifferentialPod right) {
        opMode.telemetry.addLine(started() ? "FINISHING ALIGNMENT — drive waiting" : "INIT — ALIGNING PODS TO FORWARD");
        opMode.telemetry.addData("Left", "%s | %.4f V | error %.1f deg",
                left.alignmentStatus(), left.absoluteVoltage(), left.alignmentErrorDegrees());
        opMode.telemetry.addData("Right", "%s | %.4f V | error %.1f deg",
                right.alignmentStatus(), right.absoluteVoltage(), right.alignmentErrorDegrees());
        opMode.telemetry.update();
    }
    @Override public void stopAcquiredMotors() {
        List<Runnable> actions = new ArrayList<>();
        for (DcMotorEx motor : motors) if (motor != null) actions.add(() -> motor.setVelocity(0.0));
        Cleanup.runAll(actions.toArray(new Runnable[0]));
    }
    @Override public void close() {
        List<Runnable> actions = new ArrayList<>();
        actions.add(this::stopAcquiredMotors);
        for (DcMotorEx motor : motors) if (motor != null)
            actions.add(() -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        actions.add(hubs::close);
        Cleanup.runAll(actions.toArray(new Runnable[0]));
    }
}
