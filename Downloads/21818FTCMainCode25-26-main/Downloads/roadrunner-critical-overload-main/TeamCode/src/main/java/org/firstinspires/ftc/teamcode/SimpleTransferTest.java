package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name="SimpleTransfer + ShooterVel + LLSeed + Pinpoint + Turret + Hood", group="Test")
public class SimpleTransferTest extends LinearOpMode {

    // ===== Transfer test hardware =====
    private Servo transferMover;
    private CRServo leftTransfer;
    private CRServo rightTransfer;

    // Shooter motor (ENCODER velocity control)
    private DcMotorEx shootMotor;

    // Hood servo
    private Servo hoodServo;

    // ===== Hood hard limits (your marked values) =====
    private static final double HOOD_MIN = 0.3777;
    private static final double HOOD_MAX = 0.6579;

    // ===== Shooter velocity presets =====
    private static final double SHOOT_VELO_SMALL_TPS  = 1590.0;
    private static final double SHOOT_VELO_MIDDLE_TPS = 1780.0;
    private static final double SHOOT_VELO_LARGE_TPS  = 2800.0;

    // Current selected shooter velocity (ticks/second)
    private double shootVeloTPS = SHOOT_VELO_LARGE_TPS;

    // ===== Turret =====
    private DcMotorEx turretMotor;

    // ===== Blue goal (AprilTag 20) coordinates from ftc2025DECODE.fmap =====
    // Units must match Pinpoint pose (mm).
    private static final double GOAL_X_MM = -1482.7;
    private static final double GOAL_Y_MM = -1413.3;

    // ===== Turret mapping =====
    // ticks: 0 (left edge) -> -1596 (right edge) over 270 degrees (avoiding front blind zone)
    private static final int TURRET_MIN_TICKS = -1596; // right edge
    private static final int TURRET_MAX_TICKS = 0;     // left edge

    private static final double LEFT_EDGE_DEG  = 45.0;   // turret angle at ticks=0
    private static final double RIGHT_EDGE_DEG = 315.0;  // turret angle at ticks=-1596
    private static final double SWEEP_DEG = 270.0;

    // ticks per degree along the allowed sweep (note: negative)
    private static final double TICKS_PER_DEG = (TURRET_MIN_TICKS - TURRET_MAX_TICKS) / SWEEP_DEG; // -1596/270

    // Turret motor power while RUN_TO_POSITION
    private static final double TURRET_POWER = 0.6;

    // Drive motors (same as your second file)
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Pinpoint + Limelight
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    // Your measured offsets (keep EXACTLY from second file)
    private static final double X_POD_OFFSET_MM = 127.0;
    private static final double Y_POD_OFFSET_MM = -181.0;

    // Limelight (seed only)
    private static final int APRILTAG_PIPELINE = 2;
    private static final long MAX_STALENESS_MS = 200;
    private static final long SEED_TIMEOUT_MS = 3000; // retry window

    @Override
    public void runOpMode() {

        // ===== SimpleTransferTest hardware =====
        transferMover = hardwareMap.get(Servo.class, "flywheelMover");
        leftTransfer  = hardwareMap.get(CRServo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");

        // Shooter motor as DcMotorEx for velocity control
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Hood servo
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(HOOD_MIN); // safe start

        // ===== Turret hardware =====
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        // You said turret always starts at 0 (left edge physical).
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Use built-in position control
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_POWER);

        // ===== Drive hardware (same as second file) =====
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ===== Pinpoint (same as second file) =====
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED, // X
                GoBildaPinpointDriver.EncoderDirection.REVERSED  // Y
        );

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);

        // ===== Limelight (seed only) =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Pinpoint will be seeded ONCE from Limelight after START.");
        telemetry.addLine("Robot must be still and tags visible at start.");
        telemetry.addLine("Shooter uses encoder velocity (edit SHOOT_VELO_TPS).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 1) Reset FIRST
        pinpoint.resetPosAndIMU();

        // Small delay helps IMU + Limelight settle
        sleep(200);

        // 2) Get seed pose (retry window)
        Pose2D seedPose = null;
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < SEED_TIMEOUT_MS) {
            seedPose = getFreshLimelightPoseOnce();
            if (seedPose != null) break;
            sleep(20);
        }

        boolean seeded = false;

        // 3) Seed SECOND (this becomes Pinpoint’s initial pose)
        if (seedPose != null) {
            pinpoint.setPosition(seedPose);
            seeded = true;

            // Optional but helpful: apply seed before you start driving
            pinpoint.update();
        }

        // Stop Limelight after seeding attempt
        limelight.stop();

        // Hood step size + edge detect
        final double HOOD_STEP = 0.005;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        // Shooter preset edge detect
        boolean lastDpadLeft = false;
        boolean lastDpadUpPreset = false;
        boolean lastDpadRight = false;

        while (opModeIsActive()) {

            // ===== Shooter preset buttons (dpad) =====
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadUpPreset = gamepad1.dpad_up;
            boolean dpadRight = gamepad1.dpad_right;

            if (dpadLeft && !lastDpadLeft) {
                shootVeloTPS = SHOOT_VELO_SMALL_TPS;
                hoodServo.setPosition(0.38);
            }
            if (dpadUpPreset && !lastDpadUpPreset) {
                shootVeloTPS = SHOOT_VELO_MIDDLE_TPS;
                hoodServo.setPosition(0.38);
            }
            if (dpadRight && !lastDpadRight) {
                shootVeloTPS = SHOOT_VELO_LARGE_TPS;
                hoodServo.setPosition(0.6);
            }

            lastDpadLeft = dpadLeft;
            lastDpadUpPreset = dpadUpPreset;
            lastDpadRight = dpadRight;

            // ===== Transfer Mover Positions (same behavior as your simple test) =====
            if (gamepad1.a) {
                transferMover.setPosition(0.5);
            }
            if (gamepad1.b) {
                transferMover.setPosition(0.0814);
            }

            // ===== Spin Transfer Servos Opposite Directions (same) =====
            if (gamepad1.x) {
                leftTransfer.setPower(-1.0);
                rightTransfer.setPower( 1.0);
            } else {
                leftTransfer.setPower(0);
                rightTransfer.setPower(0);
            }

            // ===== Shooter motor: encoder velocity while holding Y =====
            if (gamepad1.y) {
                // Keeping your original "reverse" intent by commanding negative velocity.
                shootMotor.setVelocity(-shootVeloTPS);
            } else {
                shootMotor.setVelocity(0);
            }

            // ===== Hood servo adjust (min/max clamped) =====
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;

            if (dpadUp && !lastDpadUp) {
                hoodServo.setPosition(clamp(hoodServo.getPosition() + HOOD_STEP, HOOD_MIN, HOOD_MAX));
            }
            if (dpadDown && !lastDpadDown) {
                hoodServo.setPosition(clamp(hoodServo.getPosition() - HOOD_STEP, HOOD_MIN, HOOD_MAX));
            }

            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;

            // ===== Driver input (same mecanum math as second file) =====
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double rotate  =  gamepad1.right_stick_x;

            double fl = forward + strafe + rotate;
            double fr = forward - strafe - rotate;
            double bl = forward - strafe + rotate;
            double br = forward + strafe - rotate;

            double max = Math.max(
                    Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br))
            );
            if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ===== Pinpoint update (continuous pose) =====
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            // ===== Auto turret aim at blue goal (Tag 20) =====
            double robotX = pose.getX(DistanceUnit.MM);
            double robotY = pose.getY(DistanceUnit.MM);
            double robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

            // Angle from robot to goal in FIELD frame
            double dx = GOAL_X_MM - robotX;
            double dy = GOAL_Y_MM - robotY;
            double goalFieldDeg = Math.toDegrees(Math.atan2(dy, dx));   // [-180,180]

            // Convert to ROBOT-relative turret angle (0-360)
            double desiredTurretDeg = norm360(goalFieldDeg - robotHeadingDeg);

            // Convert to ticks with blind-zone avoidance + limits
            int turretTargetTicks = turretTicksFromAngle(desiredTurretDeg);

            // Command turret
            turretMotor.setTargetPosition(turretTargetTicks);
            // (Mode already RUN_TO_POSITION)
            turretMotor.setPower(TURRET_POWER);

            // ===== Telemetry (keep your existing pose/turret telemetry + add shooter/hood) =====
            telemetry.addData("GoalFieldDeg", "%.1f", goalFieldDeg);
            telemetry.addData("TurretDesiredDeg(0-360)", "%.1f", desiredTurretDeg);
            telemetry.addData("TurretTargetTicks", turretTargetTicks);
            telemetry.addData("TurretTicksNow", turretMotor.getCurrentPosition());

            telemetry.addData("Seeded", seeded ? "YES" : "NO (no fresh LL botpose)");
            telemetry.addData("X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
            telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));

            telemetry.addData("ShooterTargetVel(tps)", "%.0f", shootVeloTPS);
            telemetry.addData("ShooterVelNow(tps)", "%.0f", shootMotor.getVelocity());
            telemetry.addData("HoodPos", "%.4f (min %.4f max %.4f)", hoodServo.getPosition(), HOOD_MIN, HOOD_MAX);

            telemetry.update();
        }
    }

    // ---------------- Limelight helpers (seed only) ----------------
    private Pose2D waitForFreshLimelightPose(long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeoutMs) {
            Pose2D p = getFreshLimelightPoseOnce();
            if (p != null) return p;
            sleep(20);
        }
        return null;
    }

    private Pose2D getFreshLimelightPoseOnce() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return null;
        if (r.getStaleness() > MAX_STALENESS_MS) return null;

        Pose3D p = r.getBotpose();
        if (p == null) return null;

        return toPose2D_MM(p);
    }

    private Pose2D toPose2D_MM(Pose3D botpose) {
        Position p = botpose.getPosition();
        return new Pose2D(
                DistanceUnit.MM,
                DistanceUnit.METER.toMm(p.x),
                DistanceUnit.METER.toMm(p.y),
                AngleUnit.DEGREES,
                botpose.getOrientation().getYaw(AngleUnit.DEGREES)
        );
    }

    // Normalize angle to [0, 360)
    private double norm360(double deg) {
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    // Clamp to [min,max]
    private int clampInt(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Convert desired turret angle (0-360, robot-relative) to encoder ticks,
    // while enforcing blind zone avoidance and mechanical limits.
    private int turretTicksFromAngle(double desiredDeg0to360) {
        double a = norm360(desiredDeg0to360);

        // Blind zone is front 90 degrees: [315,360) U [0,45)
        // Allowed zone is [45,315]
        if (a < LEFT_EDGE_DEG) {
            a = LEFT_EDGE_DEG;       // snap to left edge
        } else if (a > RIGHT_EDGE_DEG) {
            a = RIGHT_EDGE_DEG;      // snap to right edge
        }

        // Map angle -> ticks:
        // angle 45 => ticks 0
        // angle 315 => ticks -1596
        double ticksD = (a - LEFT_EDGE_DEG) * TICKS_PER_DEG + TURRET_MAX_TICKS;
        int ticks = (int)Math.round(ticksD);

        return clampInt(ticks, TURRET_MIN_TICKS, TURRET_MAX_TICKS);
    }

    // Double clamp helper
    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}