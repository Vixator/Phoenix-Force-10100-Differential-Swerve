package org.firstinspires.ftc.teamcode;

import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

/** One source of truth for Pinpoint identity, geometry, and commissioning gates. */
public final class PinpointSettings {
    public enum HeadingConvention { UNVERIFIED, COUNTERCLOCKWISE_POSITIVE }

    public static final String NAME = "pinpoint";
    public static final double X_POD_OFFSET_MM = 199.25;
    public static final double Y_POD_OFFSET_MM = 88.0;
    public static final GoBildaPinpointDriver.EncoderDirection X_DIRECTION_CANDIDATE =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection Y_DIRECTION_CANDIDATE =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Commissioned hardware values: X/Y directions and CCW-positive heading verified on the robot.
    public static final boolean DIRECTIONS_VERIFIED = true;
    public static final HeadingConvention HEADING_CONVENTION = HeadingConvention.COUNTERCLOCKWISE_POSITIVE;

    private PinpointSettings() { }

    public static PinpointConfig diagnosticConfig() {
        return new PinpointConfig(config -> {
            config.name.set(NAME);
            config.xPodDirection.set(X_DIRECTION_CANDIDATE);
            config.yPodDirection.set(Y_DIRECTION_CANDIDATE);
            config.xPodOffset.set(X_POD_OFFSET_MM);
            config.yPodOffset.set(Y_POD_OFFSET_MM);
            config.offsetUnits.set(DistanceUnit.MM);
            config.globalDistanceUnit.set(DistanceUnit.INCH);
            config.encoderResolutionUnit.set(DistanceUnit.MM);
            config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            config.ticksPerUnit.set(OptionalDouble.empty());
            config.resetMode.set(PinpointLocalizer.ResetMode.NONE);
        });
    }

    public static PinpointConfig poweredConfig() {
        requirePoweredVerified();
        return diagnosticConfig();
    }

    public static void requirePoweredVerified() {
        if (!DIRECTIONS_VERIFIED
                || HEADING_CONVENTION != HeadingConvention.COUNTERCLOCKWISE_POSITIVE) {
            throw new IllegalStateException("Pinpoint directions/heading are not commissioned");
        }
    }
}
