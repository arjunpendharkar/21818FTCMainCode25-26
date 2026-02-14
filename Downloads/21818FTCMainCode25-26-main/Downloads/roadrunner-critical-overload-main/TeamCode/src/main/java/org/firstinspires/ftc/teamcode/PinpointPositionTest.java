package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Step13 Drive + Pinpoint Test", group="Test")
public class PinpointPositionTest extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Pinpoint
    private GoBildaPinpointDriver pinpoint;

    // Your measured offsets (already validated)
    private static final double X_POD_OFFSET_MM = 127.0;
    private static final double Y_POD_OFFSET_MM = -181.0;

    @Override
    public void runOpMode() {

        // ===== Drive hardware =====
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Typical mecanum setup
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ===== Pinpoint =====
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED, // X
                GoBildaPinpointDriver.EncoderDirection.REVERSED   // Y
        );

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);

        telemetry.addLine("STEP 13: Drive a loop, return to start.");
        telemetry.addLine("Robot MUST be still when you press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Reset pose + IMU at start of test
        pinpoint.resetPosAndIMU();

        while (opModeIsActive()) {

            // ===== Driver input =====
            double forward = -gamepad1.left_stick_y;   // forward/back
            double strafe  =  gamepad1.left_stick_x;   // left/right
            double rotate  =  gamepad1.right_stick_x;  // turn

            // ===== Mecanum math =====
            double fl = forward + strafe + rotate;
            double fr = forward - strafe - rotate;
            double bl = forward - strafe + rotate;
            double br = forward + strafe - rotate;

            // Normalize
            double max = Math.max(
                    Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br))
            );
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            // Set motors
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ===== Pinpoint update =====
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            telemetry.addData("X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
            telemetry.addData("Heading (deg)", "%.1f",
                    pose.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}