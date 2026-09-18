package org.firstinspires.ftc.teamcode;

/** Bounded analog-feedback pod alignment with no hardware dependencies. */
public final class PodAlignmentController {
    public static final double KP = 0.005;
    public static final double MAX_COMMAND = 0.20;
    public static final double TOLERANCE_DEGREES = 2.0;
    public static final double SETTLE_SECONDS = 0.10;
    public static final double TIMEOUT_SECONDS = 5.0;

    private final double targetDegrees;
    private final int analogSign;
    private boolean active;
    private boolean complete;
    private boolean failed;
    private double elapsedSeconds;
    private double settledSeconds;
    private boolean wasInTolerance;
    private double lastErrorDegrees;
    private double command;
    private String status = "idle";

    public PodAlignmentController(double targetDegrees, int analogSign) {
        if (!SwervePodEncoder.validForward(targetDegrees)) {
            throw new IllegalArgumentException("Invalid analog alignment target");
        }
        if (analogSign != 1 && analogSign != -1) {
            throw new IllegalArgumentException("Analog sign must be +1 or -1");
        }
        this.targetDegrees = targetDegrees;
        this.analogSign = analogSign;
    }

    public void start() {
        active = true;
        complete = false;
        failed = false;
        elapsedSeconds = 0.0;
        settledSeconds = 0.0;
        wasInTolerance = false;
        lastErrorDegrees = 0.0;
        command = 0.0;
        status = "active";
    }

    public void abort(String reason) {
        active = false;
        complete = false;
        failed = true;
        command = 0.0;
        status = reason;
    }

    public void step(double volts, double seconds) {
        if (!active) return;
        if (!SwervePodEncoder.validVoltage(volts)) {
            abort(SwervePodEncoder.voltageFault(volts));
            return;
        }
        if (!Double.isFinite(seconds) || seconds < 0.0) {
            abort("invalid sample time");
            return;
        }
        elapsedSeconds += seconds;
        lastErrorDegrees = SwervePodEncoder.analogErrorDegrees(volts, targetDegrees, analogSign);
        boolean inTolerance = Math.abs(lastErrorDegrees) <= TOLERANCE_DEGREES;
        settledSeconds = inTolerance && wasInTolerance ? settledSeconds + seconds : 0.0;
        wasInTolerance = inTolerance;
        if (elapsedSeconds >= TIMEOUT_SECONDS) {
            abort("timeout");
        } else if (settledSeconds >= SETTLE_SECONDS) {
            active = false;
            complete = true;
            command = 0.0;
            status = "target reached";
        } else if (inTolerance) {
            command = 0.0;
            status = "settling";
        } else {
            status = "active";
            // The confirmed motor wiring makes positive steering command rotate CCW,
            // while positive analog error requires a CW correction.
            command = clamp(DifferentialSwervePodController.motorSteeringForClockwise(
                    lastErrorDegrees * KP), -MAX_COMMAND, MAX_COMMAND);
        }
    }

    public boolean isActive() { return active; }
    public boolean isComplete() { return complete; }
    public boolean isFailed() { return failed; }
    public double getTargetDegrees() { return targetDegrees; }
    public double getErrorDegrees() { return lastErrorDegrees; }
    public double getCommand() { return command; }
    public String getStatus() { return status; }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
