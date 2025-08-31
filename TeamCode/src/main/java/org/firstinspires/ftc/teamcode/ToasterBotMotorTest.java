package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// FTC Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Odometry + Drive Test")
public class ToasterBotMotorTest extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    // FTC Dashboard
    private FtcDashboard dashboard;

    private BNO055IMU imu;
    private Orientation angles;

    @Override
    public void runOpMode() {

        // Initialize Dashboard
        dashboard = FtcDashboard.getInstance();

        // Map motors
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear = hardwareMap.get(DcMotor.class, "backLeft");
        rightRear = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse one side if needed
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run using encoder (important for odometry tracking)
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParameters);

        waitForStart();

        while (opModeIsActive()) {
            // --------------------
            // DRIVE CODE (example: tank drive)
            double drive = gamepad1.left_stick_y; // forward/back
            double strafe = -gamepad1.left_stick_x; // left/right
            double turn = -gamepad1.right_stick_x;  // rotate

            double lfPower = drive + strafe + turn;
            double rfPower = drive - strafe - turn;
            double lrPower = drive - strafe + turn;
            double rrPower = drive + strafe - turn;

            leftFront.setPower(lfPower);
            rightFront.setPower(rfPower);
            leftRear.setPower(lrPower);
            rightRear.setPower(rrPower);
            // --------------------

            // ODOMETRY READING
            int leftOdometry = leftRear.getCurrentPosition();   // Left pod
            int rightOdometry = rightFront.getCurrentPosition(); // Right pod
            int centerOdometry = leftFront.getCurrentPosition();  // Center pod

            double x = -(centerOdometry / 1000.0);  // horizontal movement
            double y = -((leftOdometry + rightOdometry) / 2000.0);

            double heading = getHeadingRadians();

            // DS Telemetry
            telemetry.addData("Left Pod", leftOdometry);
            telemetry.addData("Right Pod", rightOdometry);
            telemetry.addData("Center Pod", centerOdometry);
            telemetry.addData("Heading (rad)", heading);
            telemetry.update();

            // FTC Dashboard Telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Left Pod", leftOdometry);
            packet.put("Right Pod", rightOdometry);
            packet.put("Center Pod", centerOdometry);
            packet.put("X", x);
            packet.put("Y", y);
            packet.put("Heading (rad)", heading);

            packet.fieldOverlay()
                    .setFill("blue")
                    .fillCircle(x, y, 3);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    private double getHeadingRadians() {
        return imu.getAngularOrientation().firstAngle;
    }
}
