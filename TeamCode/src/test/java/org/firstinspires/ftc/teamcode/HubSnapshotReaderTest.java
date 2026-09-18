package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class HubSnapshotReaderTest {
    private static class Fixture {
        long now = 1;
        boolean stopped;
        int reads;
        int stops;
        final List<String> events = new ArrayList<>();

        HubSnapshotReader reader(HubSnapshotReader.Source source) {
            return new HubSnapshotReader(() -> {
                reads++;
                events.add("read");
                now += 5_000_000;
                return source.refresh();
            }, () -> {
                stops++;
                events.add("stop");
            }, () -> {
                events.add("pause");
                now += 10_000_000;
            }, () -> stopped, () -> now);
        }
    }

    @Test
    public void validSnapshotNeedsNoStopOrRetry() {
        Fixture f = new Fixture();
        HubSnapshotReader reader = f.reader(() -> null);
        assertEquals(5_000_001, reader.read());
        assertEquals(Arrays.asList("read"), f.events);
        assertEquals(0, reader.getRecoveredReads());
    }

    @Test
    public void transientFailureStopsBeforeRetryAndReturnsOnlyFreshCompleteSnapshot() {
        Fixture f = new Fixture();
        HubSnapshotReader reader = f.reader(() -> f.reads == 1 ? "module 173 fake" : null);
        assertEquals(20_000_001, reader.read());
        assertEquals(Arrays.asList("read", "stop", "pause", "read"), f.events);
        assertEquals(1, reader.getRecoveredReads());
        assertEquals("module 173 fake", reader.getLastFailure());
        reader.read();
        assertEquals(1, f.stops);
        assertEquals(1, reader.getRecoveredReads());
    }

    @Test
    public void persistentFailureHasBoundedAttemptsAndIdentifiesHub() {
        Fixture f = new Fixture();
        HubSnapshotReader reader = f.reader(() -> "Expansion Hub module 173 fake");
        HubSnapshotReader.FeedbackFault fault = assertThrows(HubSnapshotReader.FeedbackFault.class, reader::read);
        assertTrue(fault.getMessage().contains("module 173"));
        assertEquals(3, f.reads);
        assertEquals(1, f.stops);
        assertEquals(0, reader.getRecoveredReads());
    }

    @Test
    public void slowFailureDoesNotStartAnotherAttempt() {
        Fixture f = new Fixture();
        HubSnapshotReader reader = f.reader(() -> {
            f.now += HubSnapshotReader.RECOVERY_NANOS;
            return "hub timeout";
        });
        assertThrows(HubSnapshotReader.FeedbackFault.class, reader::read);
        assertEquals(Arrays.asList("read", "stop"), f.events);
    }

    @Test
    public void validRetryArrivingAfterDeadlineDoesNotResume() {
        Fixture f = new Fixture();
        HubSnapshotReader reader = f.reader(() -> {
            if (f.reads == 1) return "fake";
            f.now += HubSnapshotReader.RECOVERY_NANOS;
            return null;
        });
        assertThrows(HubSnapshotReader.FeedbackFault.class, reader::read);
        assertEquals(0, reader.getRecoveredReads());
    }

    @Test
    public void stopCancelsWithoutReturningFakeDataOrRetrying() {
        Fixture f = new Fixture();
        HubSnapshotReader reader = f.reader(() -> {
            f.stopped = true;
            return "fake";
        });
        assertEquals(0, reader.read());
        assertEquals(1, f.reads);
        assertEquals(0, reader.read());
        assertEquals(1, f.reads);
    }

    @Test
    public void stopDeliveryFailurePreventsRecovery() {
        HubSnapshotReader reader = new HubSnapshotReader(() -> "fake", () -> {
            throw new IllegalStateException("motor unavailable");
        }, () -> { throw new AssertionError("Must not retry after failed motor stop"); }, () -> false, () -> 1L);
        HubSnapshotReader.FeedbackFault fault = assertThrows(HubSnapshotReader.FeedbackFault.class, reader::read);
        assertTrue(fault.getMessage().contains("motor stop failed"));
    }

    @Test
    public void recoveryStopClearsSlewOutputBeforeNextCommand() {
        DifferentialSwervePodController controller = new DifferentialSwervePodController();
        controller.update(0, 0, Math.PI / 4, 1, 0.2, 0.5, 0.01, 1, 0.2, 2);
        assertTrue(Math.abs(controller.getSteeringCommand()) > 0.02);
        controller.stopOutput();
        assertEquals(0.0, controller.getLeftMotorCommand(), 0.0);
        assertEquals(0.0, controller.getRightMotorCommand(), 0.0);
        controller.update(0, 0, Math.PI / 4, 0, 0.01, 0.5, 0.01, 1, 0.2, 2);
        assertEquals(-0.02, controller.getSteeringCommand(), 1e-10);
    }
}
