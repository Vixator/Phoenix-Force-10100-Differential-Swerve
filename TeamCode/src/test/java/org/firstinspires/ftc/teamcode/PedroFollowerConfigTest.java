package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

public class PedroFollowerConfigTest {
    @Test public void syntheticConfigurationPopulatesEveryRequiredModelValue() {
        ForesightConfig config = PedroFollowerConfig.syntheticForTests();
        PedroFollowerConfig.validateResolved(config);
        assertEquals(2, config.linearBrakeCoefficients.get().numRows());
        assertEquals(2, config.linearBrakeCoefficients.get().numCols());
        assertEquals(1000.0, config.timeoutConstraint.get(), 0.0);
        assertEquals(0.5, config.translationalConstraint.get(), 0.0);
        assertEquals(Math.toRadians(3.0), config.headingConstraint.get(), 1e-12);
        assertFalse(config.cosineScale.get());
        new Foresight(config);
    }

    @Test(expected = IllegalStateException.class)
    public void productionModelRemainsClosedUntilMeasurementsAreRecorded() {
        PedroFollowerConfig.create();
    }

    @Test public void commissioningEnvelopeIsInternallyConsistent() {
        PedroDriveConfig.validate();
        assertEquals(0.15, PedroDriveConfig.AUTONOMOUS_MAX_DRIVE, 0.0);
        assertEquals(0.20, PedroDriveConfig.AUTONOMOUS_TURN_LIMIT, 0.0);
    }
}
