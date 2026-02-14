package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Hood Servo Test", group="Test")
public class HoodServoTest extends LinearOpMode {

    private Servo hoodServo;

    // Change this to match your Configuration name
    private static final String HOOD_SERVO_NAME = "hoodServo";

    // Optional: limit travel so you don't slam into hard stops while testing
    private static final double MIN_POS = 0.05;
    private static final double MAX_POS = 0.95;

    @Override
    public void runOpMode() {

        hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);

        double pos = 0.5; // start in the middle
        hoodServo.setPosition(pos);

        telemetry.addLine("Ready. Move left stick Y to move hood.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Left stick Y is -1 (up) to +1 (down)
            // We want: stick up => higher position, so invert it
            double stick = -gamepad1.left_stick_y;

            // Map [-1..+1] -> [0..1]
            pos = (stick + 1.0) / 2.0;

            // Clamp to safe range
            pos = Range.clip(pos, MIN_POS, MAX_POS);

            hoodServo.setPosition(pos);

            telemetry.addData("Stick", stick);
            telemetry.addData("Servo Position Command", pos);
            telemetry.update();
        }
    }
}