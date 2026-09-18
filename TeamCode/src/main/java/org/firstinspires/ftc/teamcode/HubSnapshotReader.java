package org.firstinspires.ftc.teamcode;

import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

/** Bounded whole-snapshot retry; no fake/partial snapshot is ever returned as valid. */
public final class HubSnapshotReader {
    public static final int MAX_ATTEMPTS = 3;
    public static final long RECOVERY_NANOS = 150_000_000L;
    public static final int RETRY_DELAY_MS = 10;

    public interface Source {
        /** Refresh every required hub. Return null on success, otherwise the failed hub details. */
        String refresh();
    }

    public static final class FeedbackFault extends RuntimeException {
        public FeedbackFault(String message) { super(message); }
    }

    private final Source source;
    private final Runnable stopMotors;
    private final Runnable pause;
    private final BooleanSupplier stopped;
    private final LongSupplier clock;
    private int recoveredReads;
    private String lastFailure = "none";

    public HubSnapshotReader(Source source, Runnable stopMotors, Runnable pause,
                             BooleanSupplier stopped, LongSupplier clock) {
        this.source = source;
        this.stopMotors = stopMotors;
        this.pause = pause;
        this.stopped = stopped;
        this.clock = clock;
    }

    /** Returns a valid snapshot timestamp, or zero when Stop cancels the read. */
    public long read() {
        long started = clock.getAsLong();
        for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
            if (stopped.getAsBoolean()) return 0;
            String failure = source.refresh();
            long sampled = clock.getAsLong();
            if (stopped.getAsBoolean()) return 0;
            if (failure == null) {
                if (attempt > 1) {
                    if (sampled - started >= RECOVERY_NANOS) break;
                    recoveredReads++;
                }
                return sampled;
            }
            lastFailure = failure;
            // Stop on the FIRST failed read, before sleeping or contacting another hub.
            if (attempt == 1) {
                try {
                    stopMotors.run();
                } catch (RuntimeException exception) {
                    throw new FeedbackFault(lastFailure + " | motor stop failed: " + exception);
                }
            }
            if (attempt == MAX_ATTEMPTS || clock.getAsLong() - started >= RECOVERY_NANOS) break;
            pause.run();
            if (clock.getAsLong() - started >= RECOVERY_NANOS) break;
        }
        if (stopped.getAsBoolean()) return 0;
        throw new FeedbackFault("Hub feedback unavailable after bounded retries: " + lastFailure);
    }

    public int getRecoveredReads() { return recoveredReads; }
    public String getLastFailure() { return lastFailure; }
}
