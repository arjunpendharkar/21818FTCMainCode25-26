package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "MAINMIDDLETeleop", group = "Test")
public class SimpleDriveShooterIntake extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotor shootMotor;

    private CRServo rightTransfer, leftTransfer, spindexServo;
    private Servo flywheelMover;

    // ---------------- PINPOINT + LIMELIGHT ----------------
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    private static final double PINPOINT_OFFSET_DX_IN = -6.5;
    private static final double PINPOINT_OFFSET_DY_IN = -5.75;

    private static final int APRILTAG_PIPELINE = 2;
    private static final long MAX_STALENESS_MS = 200;

    // Field-centric tuning
    private static final int HEADING_SIGN = -1;
    private static final double HEADING_OFFSET_DEG = 180.0;

    // ---------------- CONSTANTS ----------------
    private static final double SHOOT_POWER = 0.71;

    private static final double TRANSFER_RIGHT_PWR = +1.0;
    private static final double TRANSFER_LEFT_PWR  = -1.0;

    private static final double FULL_OUT   = 0.10;
    private static final double INSIDE_POS = 0.30;

    // Spindex (P2 hold)
    private static final double SPINDEX_HOLD_POWER = 0.15;

    // Intake
    private static final double INTAKE_POWER = 0.80;

    // Drive
    private static final double DEADBAND = 0.05;
    private static final double SLOW_MULT = 0.50; // 50% speed (P1 RT)

    // ---------------- STATE ----------------
    private boolean shooterSystemOn = false;
    private boolean lastRTClickP2 = false;

    private int intakeMode = 0; // 0=off, +1=in, -1=out
    private boolean lastLB1 = false;
    private boolean lastRB1 = false;

    // drive mode + reseed button edges
    private boolean fieldCentricEnabled = false; // ✅ STARTS NORMAL
    private boolean lastA1 = false;
    private boolean lastB1 = false;

    // Reseed status
    private boolean seeded = false;
    private String lastReseedMsg = "none";

    @Override
    public void runOpMode() {

        // ---------- PINPOINT + LIMELIGHT ----------
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Drivetrain
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");


        // Mechanisms
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        shootMotor  = hardwareMap.get(DcMotor.class, "shootMotor");

        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");
        leftTransfer  = hardwareMap.get(CRServo.class, "leftTransfer");
        spindexServo  = hardwareMap.get(CRServo.class, "spindexServo");

        flywheelMover = hardwareMap.get(Servo.class, "flywheelMover");

        // Directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // BRAKE mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Safe init
        setDrivePower(0, 0, 0, 0);
        intakeMotor.setPower(0);
        shootMotor.setPower(0);
        rightTransfer.setPower(0);
        leftTransfer.setPower(0);
        spindexServo.setPower(0);
        flywheelMover.setPosition(FULL_OUT);

        // ---------- PINPOINT CONFIG ----------
        pinpoint.setOffsets(PINPOINT_OFFSET_DX_IN, PINPOINT_OFFSET_DY_IN, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // ---------- LIMELIGHT ----------
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        // Reset Pinpoint (ok to keep)
        pinpoint.resetPosAndIMU();

        telemetry.addLine("STARTS NORMAL DRIVE");
        telemetry.addLine("P1 A = reseed from Limelight (if tag visible)");
        telemetry.addLine("P1 B = toggle Field-Centric / Normal");
        telemetry.addLine("P1 RT = 50% slow mode (held)");
        telemetry.addLine("P1 LB/RB = intake toggle");
        telemetry.addLine("P2 RT = shooter toggle");
        telemetry.addLine("P2 LT = spindex hold");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ✅ NO INITIAL SEED HERE (driving starts immediately)

        while (opModeIsActive()) {

            // ---------- ODOM UPDATE ----------
            pinpoint.update();
            double heading = pinpoint.getHeading(AngleUnit.DEGREES);

            // ---------- P1 A = RESEED ----------
            boolean a1 = gamepad1.a;
            boolean a1Clicked = a1 && !lastA1;
            lastA1 = a1;

            if (a1Clicked) {
                Pose2D p = getFreshLimelightPoseOnce();
                if (p != null) {
                    pinpoint.setPosition(p);
                    seeded = true;
                    lastReseedMsg = "Reseed OK";
                } else {
                    lastReseedMsg = "Reseed FAILED (no fresh tag)";
                }
            }

            // ---------- P1 B = TOGGLE DRIVE MODE ----------
            boolean b1 = gamepad1.b;
            boolean b1Clicked = b1 && !lastB1;
            lastB1 = b1;

            if (b1Clicked) {
                fieldCentricEnabled = !fieldCentricEnabled;
            }

            // ---------- DRIVE INPUTS ----------
            double stickForward = -gamepad1.left_stick_y;
            double stickStrafe  =  gamepad1.left_stick_x;
            double rotate       =  gamepad1.right_stick_x;

            if (Math.abs(stickForward) < DEADBAND) stickForward = 0;
            if (Math.abs(stickStrafe)  < DEADBAND) stickStrafe  = 0;
            if (Math.abs(rotate)       < DEADBAND) rotate       = 0;

            double forward, strafe;

            if (fieldCentricEnabled) {
                double h = Math.toRadians(HEADING_SIGN * (heading + HEADING_OFFSET_DEG));
                double cos = Math.cos(h);
                double sin = Math.sin(h);

                double xField = stickStrafe;
                double yField = stickForward;

                double xRobot = xField * cos - yField * sin;
                double yRobot = xField * sin + yField * cos;

                forward = yRobot;
                strafe  = xRobot;
            } else {
                forward = stickForward;
                strafe  = stickStrafe;
            }

            double fl = forward + strafe + rotate;
            double fr = forward - strafe - rotate;
            double bl = forward - strafe + rotate;
            double br = forward + strafe - rotate;

            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(fr),
                                    Math.max(Math.abs(bl), Math.abs(br)))));

            double speedMult = (gamepad1.right_trigger > 0.5) ? SLOW_MULT : 1.0;

            setDrivePower(
                    (fl / max) * speedMult,
                    (fr / max) * speedMult,
                    (bl / max) * speedMult,
                    (br / max) * speedMult
            );

            // ---------- INTAKE TOGGLE (P1 bumpers) ----------
            boolean lb1 = gamepad1.left_bumper;
            boolean rb1 = gamepad1.right_bumper;

            boolean lb1Clicked = lb1 && !lastLB1;
            boolean rb1Clicked = rb1 && !lastRB1;

            lastLB1 = lb1;
            lastRB1 = rb1;

            if (lb1Clicked) intakeMode = (intakeMode == -1) ? 0 : -1;
            if (rb1Clicked) intakeMode = (intakeMode == +1) ? 0 : +1;

            if (intakeMode == +1) intakeMotor.setPower(+INTAKE_POWER);
            else if (intakeMode == -1) intakeMotor.setPower(-INTAKE_POWER);
            else intakeMotor.setPower(0);

            // ---------- SHOOTER SYSTEM (P2 RT toggle) ----------
            boolean rtNowP2 = gamepad2.right_trigger > 0.60;
            boolean rtClickedP2 = rtNowP2 && !lastRTClickP2;
            lastRTClickP2 = rtNowP2;

            if (rtClickedP2) {
                shooterSystemOn = !shooterSystemOn;

                if (shooterSystemOn) {
                    shootMotor.setPower(SHOOT_POWER);
                    rightTransfer.setPower(TRANSFER_RIGHT_PWR);
                    leftTransfer.setPower(TRANSFER_LEFT_PWR);
                    flywheelMover.setPosition(INSIDE_POS);
                } else {
                    shootMotor.setPower(0);
                    rightTransfer.setPower(0);
                    leftTransfer.setPower(0);
                    flywheelMover.setPosition(FULL_OUT);
                }
            }

            // ---------- SPINDEX (P2 LT hold) ----------
            spindexServo.setPower(gamepad2.left_trigger > 0.60 ? SPINDEX_HOLD_POWER : 0);

            telemetry.addData("Drive Mode", fieldCentricEnabled ? "FIELD-CENTRIC" : "NORMAL");
            telemetry.addData("Seeded", seeded ? "YES" : "NO");
            telemetry.addData("Reseed Msg", lastReseedMsg);
            telemetry.addData("Heading (deg)", "%.1f", heading);
            telemetry.update();
        }

        // Stop everything
        setDrivePower(0, 0, 0, 0);
        intakeMotor.setPower(0);
        shootMotor.setPower(0);
        rightTransfer.setPower(0);
        leftTransfer.setPower(0);
        spindexServo.setPower(0);
        flywheelMover.setPosition(FULL_OUT);

        limelight.stop();
    }

    // ---------------- LIMELIGHT HELPERS ----------------
    private Pose2D getFreshLimelightPoseOnce() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return null;
        if (r.getStaleness() > MAX_STALENESS_MS) return null;
        Pose3D p = r.getBotpose();
        if (p == null) return null;
        return toPose2D(p);
    }

    private Pose2D toPose2D(Pose3D botpose) {
        Position p = botpose.getPosition();
        return new Pose2D(
                DistanceUnit.INCH,
                DistanceUnit.METER.toInches(p.x),
                DistanceUnit.METER.toInches(p.y),
                AngleUnit.DEGREES,
                botpose.getOrientation().getYaw(AngleUnit.DEGREES)
        );
    }

    private void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}