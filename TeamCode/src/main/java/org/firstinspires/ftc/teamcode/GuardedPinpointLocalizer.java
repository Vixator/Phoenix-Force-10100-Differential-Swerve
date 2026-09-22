package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

/** Adds READY, timing, and finite-value checks around Pedro's Pinpoint localizer. */
public final class GuardedPinpointLocalizer implements Localizer {
    public static final class LocalizationFault extends RuntimeException {
        public LocalizationFault(String message) { super(message); }
        public LocalizationFault(String message, Throwable cause) { super(message, cause); }
    }

    interface Sensor {
        GoBildaPinpointDriver.DeviceStatus status();
        void calibrate(boolean resetPosition);
        double loopMicros();
        double frequencyHz();
    }

    private final Sensor sensor;
    private final Localizer delegate;
    private final BooleanSupplier armed;
    private final LongSupplier clock;
    private long lastUpdateNanos;

    public GuardedPinpointLocalizer(HardwareMap hardwareMap, BooleanSupplier armed) {
        this(hardwareMap, armed, System::nanoTime);
    }

    GuardedPinpointLocalizer(HardwareMap hardwareMap, BooleanSupplier armed, LongSupplier clock) {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PinpointSettings.NAME);
        // Pinpoint V2 uses the driver's controller-side validation; CRC is firmware V3+ only.
        pinpoint.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.LOCAL_TEST);
        this.sensor = new Sensor() {
            @Override public GoBildaPinpointDriver.DeviceStatus status() { return pinpoint.getDeviceStatus(); }
            @Override public void calibrate(boolean resetPosition) {
                if (resetPosition) pinpoint.resetPosAndIMU(); else pinpoint.recalibrateIMU();
            }
            @Override public double loopMicros() { return pinpoint.getLoopTime(); }
            @Override public double frequencyHz() { return pinpoint.getFrequency(); }
        };
        this.delegate = new PinpointLocalizer(hardwareMap, PinpointSettings.diagnosticConfig());
        this.armed = armed;
        this.clock = clock;
    }

    GuardedPinpointLocalizer(Localizer delegate, Sensor sensor, BooleanSupplier armed, LongSupplier clock) {
        this.delegate = delegate;
        this.sensor = sensor;
        this.armed = armed;
        this.clock = clock;
    }

    public void calibrateStationary(LinearOpMode opMode, boolean resetPosition) {
        calibrateStationary(opMode::isStopRequested, () -> opMode.sleep(10), resetPosition);
    }

    void calibrateStationary(BooleanSupplier stopped, Runnable pause, boolean resetPosition) {
        if (armed.getAsBoolean()) throw new IllegalStateException("Cannot calibrate Pinpoint while armed");
        if (stopped.getAsBoolean()) throw new LocalizationFault("Stop requested before Pinpoint calibration");
        sensor.calibrate(resetPosition);
        long deadline = clock.getAsLong()
                + (long) (PedroDriveConfig.PINPOINT_READY_TIMEOUT_SECONDS * 1e9);
        while (!stopped.getAsBoolean() && clock.getAsLong() < deadline) {
            long started = clock.getAsLong();
            delegate.update();
            long completed = clock.getAsLong();
            DifferentialSwerveRuntime.validateSnapshotAge(started, completed);
            if (stopped.getAsBoolean()) throw new LocalizationFault("Stop requested during Pinpoint calibration");
            if (sensor.status() == GoBildaPinpointDriver.DeviceStatus.READY) {
                validateState(delegate.state());
                lastUpdateNanos = clock.getAsLong();
                return;
            }
            pause.run();
        }
        throw new LocalizationFault("Pinpoint did not become READY within 3 seconds; status="
                + sensor.status());
    }

    public void requireReady() {
        update();
    }

    @Override
    public void update() {
        long started = clock.getAsLong();
        try {
            delegate.update();
        } catch (RuntimeException exception) {
            throw new LocalizationFault("Pinpoint update failed", exception);
        }
        long completed = clock.getAsLong();
        DifferentialSwerveRuntime.validateSnapshotAge(started, completed);
        if (sensor.status() != GoBildaPinpointDriver.DeviceStatus.READY) {
            throw new LocalizationFault("Pinpoint feedback invalid: " + sensor.status());
        }
        validateState(delegate.state());
        lastUpdateNanos = completed;
    }

    static void validateState(MotionState state) {
        if (state == null) throw new LocalizationFault("Pinpoint returned no motion state");
        Pose pose = state.pose();
        Velocity velocity = state.velocity();
        if (pose == null || velocity == null
                || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading()) || !Double.isFinite(velocity.vx)
                || !Double.isFinite(velocity.vy) || !Double.isFinite(velocity.omega)) {
            throw new LocalizationFault("Pinpoint returned nonfinite pose or velocity");
        }
    }

    @Override public void setPose(Pose pose) {
        if (pose == null || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading())) {
            throw new IllegalArgumentException("Start pose must be finite");
        }
        delegate.setPose(pose);
        validateState(delegate.state());
    }
    @Override public MotionState state() { return delegate.state(); }

    @Override
    public void reset() {
        if (armed.getAsBoolean()) throw new IllegalStateException("Pinpoint reset is forbidden while armed");
        delegate.reset();
        update();
    }

    public GoBildaPinpointDriver.DeviceStatus status() { return sensor.status(); }
    public long lastUpdateNanos() { return lastUpdateNanos; }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new LinkedHashMap<>(delegate.debug());
        map.put("status", sensor.status());
        map.put("hostUpdateNanos", lastUpdateNanos);
        map.put("deviceLoopMicros", sensor.loopMicros());
        map.put("deviceFrequencyHz", sensor.frequencyHz());
        return map;
    }
}
