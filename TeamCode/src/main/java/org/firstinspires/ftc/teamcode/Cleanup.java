package org.firstinspires.ftc.teamcode;

/** Attempts every cleanup action and retains secondary failures on the first exception. */
final class Cleanup {
    private Cleanup() { }
    static void runAll(Runnable... actions) {
        RuntimeException failure = null;
        for (Runnable action : actions) {
            try { action.run(); }
            catch (RuntimeException exception) {
                if (failure == null) failure = exception;
                else if (failure != exception) failure.addSuppressed(exception);
            }
        }
        if (failure != null) throw failure;
    }
}
