package org.firstinspires.ftc.teamcode;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.junit.Test;
import static org.junit.Assert.assertEquals;
public class PedroPathResultTest {
    @Test public void parametricEndpointWithHeadingOrSpeedErrorFails() {
        assertEquals("END_OUTSIDE_TOLERANCE", PedroPathResult.evaluate(Pose.zero(), new Pose(0, 0, 1), Velocity.zero()));
        assertEquals("END_OUTSIDE_TOLERANCE", PedroPathResult.evaluate(Pose.zero(), Pose.zero(), new Velocity(2, 0, 0)));
        assertEquals("INVALID_TERMINAL_FEEDBACK", PedroPathResult.evaluate(Pose.zero(), Pose.zero(), new Velocity(Double.NaN, 0, 0)));
    }
    @Test public void headingWrapAndStationaryEndpointPass() {
        assertEquals("END_WITHIN_TOLERANCE", PedroPathResult.evaluate(new Pose(0, 0, Math.toRadians(179)),
                new Pose(.1, .1, Math.toRadians(-179)), Velocity.zero()));
    }
}
