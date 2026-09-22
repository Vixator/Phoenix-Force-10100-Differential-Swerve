package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Pedro Curve Test", group = "Pedro Commissioning")
public final class PedroCurveTest extends PedroPathTestBase {
    private static final Pose START = new Pose(0, 0, 0);
    @Override protected Pose startPose() { return START; }
    @Override protected Path path() {
        return Paths.curve(START, new Pose(6, 6, 0), new Pose(12, 8, 0)).constant(START);
    }
}
