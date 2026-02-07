package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Actuator 2-Pos + Shooter Toggle", group="Test")
public class ActuatorTwoPosShooterToggle extends OpMode {

    private Servo flywheelMover;   // positional mode servo
    private DcMotor shootMotor;

    // Transfer servos
    private CRServo rightTransfer, leftTransfer;

    // Adjustable preset positions (0.0 to 1.0)
    private double FULL_OUT = 0.1;
    private double PARTIAL_IN = 0.4;

    // Current commanded target position
    private double targetPos = PARTIAL_IN;

    // Toggles
    private boolean shooterOn = false;
    private boolean transferOn = false;

    private boolean lastTriggerPressed = false; // RT
    private boolean lastLeftTriggerPressed = false; // LT

    @Override
    public void init() {
        flywheelMover = hardwareMap.get(Servo.class, "flywheelMover");
        shootMotor = hardwareMap.get(DcMotor.class, "shootMotor");

        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");
        leftTransfer  = hardwareMap.get(CRServo.class, "leftTransfer");

        // Start at partial in (safe-ish default)
        targetPos = PARTIAL_IN;
        flywheelMover.setPosition(targetPos);

        shootMotor.setPower(0);

        rightTransfer.setPower(0);
        leftTransfer.setPower(0);
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // --- Servo preset buttons ---
        if (gamepad1.x) {
            targetPos = PARTIAL_IN;
        } else if (gamepad1.b) {
            targetPos = FULL_OUT;
        }

        // Command the servo
        flywheelMover.setPosition(targetPos);

        // --- Shooter toggle with right trigger ---
        boolean triggerPressed = gamepad1.right_trigger > 0.5;

        if (triggerPressed && !lastTriggerPressed) {
            shooterOn = !shooterOn; // toggle
        }
        lastTriggerPressed = triggerPressed;

        shootMotor.setPower(shooterOn ? 1.0 : 0.0);

        // --- Transfer toggle with left trigger (like your other code) ---
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.6;

        if (leftTriggerPressed && !lastLeftTriggerPressed) {
            transferOn = !transferOn;

            if (transferOn) {
                rightTransfer.setPower(+1.0);
                leftTransfer.setPower(-1.0);
            } else {
                rightTransfer.setPower(0.0);
                leftTransfer.setPower(0.0);
            }
        }
        lastLeftTriggerPressed = leftTriggerPressed;

        // --- Telemetry ---
        telemetry.addData("Servo Target", "%.3f", targetPos);
        telemetry.addData("Preset FULL_OUT", "%.3f", FULL_OUT);
        telemetry.addData("Preset PARTIAL_IN", "%.3f", PARTIAL_IN);

        telemetry.addData("Shooter", shooterOn ? "ON (1.0)" : "OFF (0.0)");
        telemetry.addData("RT", "%.2f", gamepad1.right_trigger);

        telemetry.addData("Transfer", transferOn ? "ON" : "OFF");
        telemetry.addData("LT", "%.2f", gamepad1.left_trigger);

        telemetry.update();
    }
}