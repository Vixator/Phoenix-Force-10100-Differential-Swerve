package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Pedro Line Test", group = "Pedro Commissioning")
public final class PedroLineTest extends PedroPathTestBase {
    private static final Pose START = new Pose(0, 0, 0);
    @Override protected Pose startPose() { return START; }
    @Override protected Path path() { return Paths.line(START, new Pose(12, 0, 0)).constant(START); }
}
