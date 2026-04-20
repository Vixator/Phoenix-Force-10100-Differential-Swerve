package org.firstinspires.ftc.teamcode.differentialswerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DifferentialSwervePod {
    private final String name;
    private final DcMotorEx motorA;
    private final DcMotorEx motorB;
    private final double steeringRadiansPerTick;

    private int lastMotorAPosition;
    private int lastMotorBPosition;
    private double estimatedAngleRadians;

    public DifferentialSwervePod(String name,
                                 HardwareMap hardwareMap,
                                 String motorAName,
                                 String motorBName,
                                 DcMotorSimple.Direction motorADirection,
                                 DcMotorSimple.Direction motorBDirection,
                                 double steeringRadiansPerTick) {
        this.name = name;
        this.motorA = hardwareMap.get(DcMotorEx.class, motorAName);
        this.motorB = hardwareMap.get(DcMotorEx.class, motorBName);
        this.steeringRadiansPerTick = steeringRadiansPerTick;

        motorA.setDirection(motorADirection);
        motorB.setDirection(motorBDirection);
        motorA.setZeroPowerBehavior(DifferentialSwerveConfig.ZERO_POWER_BEHAVIOR);
        motorB.setZeroPowerBehavior(DifferentialSwerveConfig.ZERO_POWER_BEHAVIOR);
        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        syncSensors();
    }

    public void syncSensors() {
        lastMotorAPosition = motorA.getCurrentPosition();
        lastMotorBPosition = motorB.getCurrentPosition();
    }

    public void setEstimatedAngleRadians(double estimatedAngleRadians) {
        this.estimatedAngleRadians = DifferentialSwerveMath.wrapAngleRadians(estimatedAngleRadians);
        syncSensors();
    }

    public void update() {
        int currentMotorAPosition = motorA.getCurrentPosition();
        int currentMotorBPosition = motorB.getCurrentPosition();

        int deltaA = currentMotorAPosition - lastMotorAPosition;
        int deltaB = currentMotorBPosition - lastMotorBPosition;
        lastMotorAPosition = currentMotorAPosition;
        lastMotorBPosition = currentMotorBPosition;

        estimatedAngleRadians += (deltaA - deltaB) * steeringRadiansPerTick;
        estimatedAngleRadians = DifferentialSwerveMath.wrapAngleRadians(estimatedAngleRadians);
    }

    public void stop() {
        motorA.setPower(0.0);
        motorB.setPower(0.0);
    }

    public void setRawDriveAndSteer(double drivePower, double steerPower) {
        double motorAPower = DifferentialSwerveMath.clamp(drivePower + steerPower, -1.0, 1.0);
        double motorBPower = DifferentialSwerveMath.clamp(drivePower - steerPower, -1.0, 1.0);
        motorA.setPower(motorAPower);
        motorB.setPower(motorBPower);
    }

    public void setDesiredState(double targetAngleRadians,
                                double targetDrivePower,
                                double steeringKp,
                                double maxDrivePower,
                                double maxSteerPower) {
        DifferentialSwerveMath.OptimizedModuleState optimizedState =
                DifferentialSwerveMath.optimize(targetAngleRadians, targetDrivePower, estimatedAngleRadians);

        double steerPower = DifferentialSwerveMath.clamp(
                optimizedState.angleErrorRadians * steeringKp,
                -maxSteerPower,
                maxSteerPower
        );

        // Reduce thrust until the wheel is mostly facing the right way.
        double headingScale = Math.max(0.0, Math.cos(optimizedState.angleErrorRadians));
        double drivePower = DifferentialSwerveMath.clamp(
                optimizedState.targetSpeed * headingScale,
                -maxDrivePower,
                maxDrivePower
        );

        setRawDriveAndSteer(drivePower, steerPower);
    }

    public String getName() {
        return name;
    }

    public double getEstimatedAngleRadians() {
        return estimatedAngleRadians;
    }

    public int getMotorAPosition() {
        return motorA.getCurrentPosition();
    }

    public int getMotorBPosition() {
        return motorB.getCurrentPosition();
    }
}
