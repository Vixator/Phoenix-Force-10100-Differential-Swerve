package org.firstinspires.ftc.teamcode;

/** Bounded analog-feedback pod alignment with no hardware dependencies. */
public final class PodAlignmentController {
    public static final double KP = 0.005;
    public static final double MAX_COMMAND = 0.20;
    public static final double TOLERANCE_DEGREES = 2.0;
    public static final double TIMEOUT_SECONDS = 5.0;

    private final double targetDegrees;
    private final int analogSign;
    private boolean active;
    private boolean complete;
    private boolean failed;
    private double elapsedSeconds;
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
        lastErrorDegrees = 0.0;
        command = 0.0;
        status = "active";
    }

    public void abort(String reason) {
        active = false;
        failed = true;
        command = 0.0;
        status = reason;
    }

    public void step(double volts, double seconds) {
        if (!active) return;
        if (!SwervePodEncoder.validVoltage(volts) || !Double.isFinite(seconds) || seconds < 0.0) {
            abort("invalid feedback");
            return;
        }
        elapsedSeconds += seconds;
        lastErrorDegrees = SwervePodEncoder.analogErrorDegrees(volts, targetDegrees, analogSign);
        if (Math.abs(lastErrorDegrees) <= TOLERANCE_DEGREES) {
            active = false;
            complete = true;
            command = 0.0;
            status = "target reached";
        } else if (elapsedSeconds >= TIMEOUT_SECONDS) {
            abort("timeout");
        } else {
            command = clamp(lastErrorDegrees * KP, -MAX_COMMAND, MAX_COMMAND);
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
