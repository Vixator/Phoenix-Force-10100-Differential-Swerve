package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;

/** Follower whose every update is enclosed by a fresh-snapshot, fail-closed hardware cycle. */
public final class SafePedroFollower extends Follower {
    public interface CycleGuard {
        double beginCycle();
        void finishCycle();
        void latchFault(String message);
        boolean hasFault();
    }

    private final CycleGuard guard;

    public SafePedroFollower(Localizer localizer, Drivetrain drivetrain,
                             Algorithm algorithm, CycleGuard guard) {
        super(localizer, drivetrain, algorithm);
        this.guard = guard;
    }

    @Override
    public void update() {
        runGuarded(null);
    }

    @Override
    public void update(double deltaTime) {
        if (!Double.isFinite(deltaTime) || deltaTime <= 0.0) {
            RuntimeException fault = new IllegalArgumentException("Follower delta time must be finite and positive");
            fail(fault);
            throw fault;
        }
        runGuarded(deltaTime);
    }

    private void runGuarded(Double requestedDelta) {
        boolean began = false;
        RuntimeException failure = null;
        try {
            if (guard.hasFault()) throw new IllegalStateException("Follower fault is latched; reinitialize");
            double measuredDelta = guard.beginCycle();
            began = true;
            if (!Double.isFinite(measuredDelta) || measuredDelta <= 0.0
                    || measuredDelta > PedroDriveConfig.MAX_FEEDBACK_AGE_SECONDS) {
                throw new IllegalArgumentException("Invalid measured follower interval");
            }
            double algorithmDelta = requestedDelta == null ? measuredDelta : requestedDelta;
            super.update(algorithmDelta);
        } catch (RuntimeException exception) {
            failure = exception;
            fail(exception);
            throw exception;
        } finally {
            if (began) {
                try { guard.finishCycle(); }
                catch (RuntimeException exception) {
                    fail(exception);
                    if (failure == null) throw exception;
                    failure.addSuppressed(exception);
                }
            }
        }
    }

    private void fail(RuntimeException exception) {
        try { super.stop(); } catch (RuntimeException stateFailure) { exception.addSuppressed(stateFailure); }
        try { drivetrain.stop(); } catch (RuntimeException stopFailure) { exception.addSuppressed(stopFailure); }
        guard.latchFault("Follower stopped: " + exception.getMessage());
    }

    @Override
    public void stop() {
        try { Cleanup.runAll(() -> super.stop(), drivetrain::stop); }
        catch (RuntimeException failure) {
            guard.latchFault("Follower stop failed: " + failure.getMessage());
            throw failure;
        }
    }
}
