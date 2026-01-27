package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Simple field-centric mecanum drive using Pinpoint heading.
 * Assumes odometry has been seeded (e.g., Limelight seed) before or at start.
 */
@TeleOp(name = "Field Centric Drive", group = "Drive")
public class FieldCentricDriveTeleOp extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;

    private static final double DEADBAND = 0.02;



    @Override
    public void runOpMode() {
        // Map drive motors
        frontLeft = hardwareMap.get(DcMotor.class, OdometryConstants.FRONT_LEFT_MOTOR_NAME);
        frontRight = hardwareMap.get(DcMotor.class, OdometryConstants.FRONT_RIGHT_MOTOR_NAME);
        backLeft = hardwareMap.get(DcMotor.class, OdometryConstants.BACK_LEFT_MOTOR_NAME);
        backRight = hardwareMap.get(DcMotor.class, OdometryConstants.BACK_RIGHT_MOTOR_NAME);

        // Direction matches existing teleop (left side reversed)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Map Pinpoint and configure offsets
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(
                OdometryConstants.PINPOINT_OFFSET_DX_INCHES,
                OdometryConstants.PINPOINT_OFFSET_DY_INCHES,
                DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU(); // start fresh; seed separately if using Limelight

        telemetry.addLine("Field-centric drive ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry every loop
            pinpoint.update();

            // Gamepad inputs: left stick = translation (field frame), right stick X = rotation
            double fieldX = applyDeadband(gamepad1.left_stick_x);      // strafe left/right
            double fieldY = applyDeadband(-gamepad1.left_stick_y);     // forward/back (invert for FTC stick)
            double turn = applyDeadband(gamepad1.right_stick_x);       // CCW +

            // Current heading from Pinpoint (radians, normalized)
            double headingRad = wrapToPi(Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES)));

            // Rotate field-centric inputs into robot frame
            double cosH = Math.cos(headingRad);
            double sinH = Math.sin(headingRad);
            double robotX = fieldX * cosH - fieldY * sinH; // robot strafe
            double robotY = fieldX * sinH + fieldY * cosH; // robot forward

            // Mecanum power mix
            double fl = robotY + robotX + turn;
            double fr = robotY - robotX - turn;
            double bl = robotY - robotX + turn;
            double br = robotY + robotX - turn;

            // Normalize to keep |power| <= 1
            double maxMag = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            fl /= maxMag;
            fr /= maxMag;
            bl /= maxMag;
            br /= maxMag;

            // Apply powers
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // Basic pose telemetry
            telemetry.addData("Pose (in)", "x=%.1f y=%.1f", pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(headingRad));
            telemetry.update();
        }
    }

    private double applyDeadband(double v) {
        return Math.abs(v) < DEADBAND ? 0.0 : v;
    }

    // Normalize angle to [-π, π]
    private double wrapToPi(double angleRad) {
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }
}
