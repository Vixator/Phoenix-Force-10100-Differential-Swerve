package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/** One bulk-cache coordinator, shared by full-drive and selected-pod diagnostics. */
final class SwerveHubSession implements AutoCloseable {
    private final HardwareMap hardwareMap;
    private final BooleanSupplier stopped;
    private List<LynxModule> hubs = new ArrayList<>();
    private LynxModule.BulkCachingMode[] previous = new LynxModule.BulkCachingMode[0];

    SwerveHubSession(HardwareMap hardwareMap, BooleanSupplier stopped) {
        this.hardwareMap = hardwareMap;
        this.stopped = stopped;
    }
    void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        previous = new LynxModule.BulkCachingMode[hubs.size()];
        for (int i = 0; i < hubs.size(); i++) previous[i] = hubs.get(i).getBulkCachingMode();
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    String refresh() {
        for (LynxModule hub : hubs) {
            if (stopped.getAsBoolean()) return "Stop requested";
            String failure = null;
            try {
                // getBulkData performs a new read and populates the SDK's cache for subsequent getters.
                if (hub.getBulkData().isFake()) failure = "SDK returned fake bulk data";
            } catch (RuntimeException exception) {
                failure = exception.getClass().getSimpleName() + ": " + exception.getMessage();
            }
            if (failure != null) {
                String detail = hardwareMap.getNamesOf(hub) + " | " + hub.getConnectionInfo()
                        + " | " + (hub.isParent() ? "parent" : "downstream") + " | " + failure;
                RobotLog.ww("DifferentialSwerve", detail);
                return detail;
            }
        }
        return null;
    }
    @Override public void close() {
        List<Runnable> actions = new ArrayList<>();
        for (int i = 0; i < previous.length; i++) {
            if (previous[i] == null) continue;
            final int index = i;
            actions.add(() -> hubs.get(index).setBulkCachingMode(previous[index]));
        }
        Cleanup.runAll(actions.toArray(new Runnable[0]));
    }
}
