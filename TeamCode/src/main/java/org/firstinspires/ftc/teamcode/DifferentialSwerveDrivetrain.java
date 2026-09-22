package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;

import java.util.LinkedHashMap;
import java.util.Map;

/** Robot-owned Pedro drivetrain adapter that preserves the tested differential-pod controller. */
public final class DifferentialSwerveDrivetrain implements Drivetrain {
    private static final double ZERO_EPSILON = 1e-12;

    private final DifferentialSwerveRuntime runtime;
    private final DifferentialPod left;
    private final DifferentialPod right;
    private final DifferentialSwerveKinematics kinematics = new DifferentialSwerveKinematics();
    private final double turnLimit;
    private DrivePowers lastPowers = DrivePowers.zero();

    public DifferentialSwerveDrivetrain(DifferentialSwerveRuntime runtime, boolean autonomous) {
        this(runtime, autonomous ? PedroDriveConfig.AUTONOMOUS_TURN_LIMIT
                : PedroDriveConfig.TELEOP_TURN_LIMIT);
    }

    DifferentialSwerveDrivetrain(DifferentialSwerveRuntime runtime, double turnLimit) {
        this.runtime = runtime;
        this.left = runtime.leftPod();
        this.right = runtime.rightPod();
        if (left == null || right == null) throw new IllegalStateException("Runtime hardware is not initialized");
        PedroDriveConfig.finiteRange("turnLimit", turnLimit, 0.0, 1.0);
        this.turnLimit = turnLimit;
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        try {
            PedroSwerveMath.requireFinite(powers, "requested");
            runtime.requireOutputAllowed();
            double turn = PedroSwerveMath.clamp(powers.turn(), -turnLimit, turnLimit);
            DrivePowers bounded = new DrivePowers(powers.forward(), powers.strafe(), turn);
            lastPowers = bounded;
            if (Math.abs(bounded.forward()) <= ZERO_EPSILON
                    && Math.abs(bounded.strafe()) <= ZERO_EPSILON
                    && Math.abs(bounded.turn()) <= ZERO_EPSILON) {
                runtime.safeZeroAll();
                runtime.markDriveRequested();
                return;
            }

            kinematics.update(bounded.forward(), PedroSwerveMath.rightFromPedroStrafe(bounded.strafe()),
                    PedroSwerveMath.clockwiseRadiansPerSecond(bounded.turn()));
            left.prepareMove(PedroSwerveMath.wheelAngle(kinematics.getLeftTargetAngle()),
                    kinematics.getLeftSpeed(), false);
            right.prepareMove(PedroSwerveMath.wheelAngle(kinematics.getRightTargetAngle()),
                    kinematics.getRightSpeed(), false);
            runtime.requireOutputAllowed();
            left.commitPrepared();
            right.commitPrepared();
            runtime.markDriveRequested();
        } catch (RuntimeException exception) {
            runtime.latchFault("Drivetrain command failed: " + exception.getMessage());
            throw exception;
        }
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        return PedroSwerveMath.maxScaling(current, delta, turnLimit);
    }

    @Override public void stop() { stop(true); }

    @Override
    public void stop(boolean brake) {
        lastPowers = DrivePowers.zero();
        try { Cleanup.runAll(runtime::safeZeroAll, left::setToBreak, right::setToBreak); }
        catch (RuntimeException exception) {
            runtime.latchFault("Drivetrain stop failed: " + exception.getMessage());
            throw exception;
        }
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new LinkedHashMap<>();
        map.put("powers", lastPowers);
        map.put("turnLimit", turnLimit);
        map.put("state", runtime.state());
        map.put("fault", runtime.fault());
        map.put("left", left.debug());
        map.put("right", right.debug());
        return map;
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return PedroSwerveMath.interpolateVelocity(xRadius, yRadius, theta);
    }
}
