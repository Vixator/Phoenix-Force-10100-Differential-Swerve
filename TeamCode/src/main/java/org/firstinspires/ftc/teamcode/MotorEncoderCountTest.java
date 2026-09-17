package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Read-only motor encoder diagnostic.
 *
 * This does not set power, velocity, direction, mode, or target position. It displays
 * raw counts while the pods are moved by hand so the known differential gear setup
 * can be checked safely. Pod angle is still read from the dedicated 1:1 pod encoder;
 * this test validates motor-pair count relationships and physical signs.
 */
@TeleOp(name = "Motor Encoder Count Test", group = "Differential Swerve")
public class MotorEncoderCountTest extends OpMode {
    private static final double GEARBOX_OUTPUT_PPR = 145.1;
    private static final double ENCODER_SHAFT_PPR = 28.0;
    private static final double PLANETARY_RATIO = 5.2;

    private DcMotor[] motors;
    private int[] zeroCounts;
    private boolean zeroCaptured;
    private boolean wasZeroPressed;

    @Override
    public void init() {
        motors = new DcMotor[]{
                hardwareMap.get(DcMotor.class, "motor0"),
                hardwareMap.get(DcMotor.class, "motor1"),
                hardwareMap.get(DcMotor.class, "motor2"),
                hardwareMap.get(DcMotor.class, "motor3")
        };
        zeroCounts = new int[motors.length];
        telemetry.setMsTransmissionInterval(100);
    }

    @Override
    public void loop() {
        int[] counts = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            counts[i] = motors[i].getCurrentPosition();
        }

        boolean zeroPressed = gamepad1.a;
        boolean captureZero = zeroPressed && !wasZeroPressed;
        wasZeroPressed = zeroPressed;
        if (!zeroCaptured || captureZero) {
            System.arraycopy(counts, 0, zeroCounts, 0, counts.length);
            zeroCaptured = true;
        }

        telemetry.addLine("READ-ONLY: no motor power, velocity, direction, mode, or reset commands.");
        telemetry.addLine("Rotate a pod by hand; press A to set a new count baseline.");
        telemetry.addLine("Motor output encoder: 145.1 pulses/rev; encoder shaft: 28 pulses/rev; gearbox: 5.2:1.");
        for (int i = 0; i < motors.length; i++) {
            int delta = counts[i] - zeroCounts[i];
            telemetry.addData("motor" + i + " raw / delta", "%d / %d", counts[i], delta);
            double gearboxOutputRevolutions = delta / GEARBOX_OUTPUT_PPR;
            double encoderShaftRevolutions = gearboxOutputRevolutions * PLANETARY_RATIO;
            telemetry.addData("motor" + i + " output rev / encoder rev", "%.4f / %.4f",
                    gearboxOutputRevolutions, encoderShaftRevolutions);
            telemetry.addData("motor" + i + " encoder shaft pulses", "%.1f",
                    encoderShaftRevolutions * ENCODER_SHAFT_PPR);
        }

        int leftDifferential = (counts[1] - zeroCounts[1]) - (counts[0] - zeroCounts[0]);
        int rightDifferential = (counts[3] - zeroCounts[3]) - (counts[2] - zeroCounts[2]);
        telemetry.addData("left pod differential delta", leftDifferential);
        telemetry.addData("right pod differential delta", rightDifferential);
        telemetry.addData("differential setup", "1:1 bevel -> 16:54 -> 50:19 common wheel gear");
        telemetry.addData("Reference", "1:1 pod encoder is authoritative for pod angle");
        telemetry.update();
    }
}
