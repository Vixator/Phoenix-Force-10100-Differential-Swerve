package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Pedro Heading Test", group = "Pedro Commissioning")
public final class PedroHeadingTest extends PedroPathTestBase {
    private static final Pose START = new Pose(0, 0, 0);
    private static final Pose END = new Pose(12, 0, Math.toRadians(45));
    @Override protected Pose startPose() { return START; }
    @Override protected Path path() { return Paths.line(START, END).linear(START, END); }
}
