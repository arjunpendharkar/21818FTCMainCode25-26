package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "LL Seed Once -> Pinpoint Odom + Dashboard", group = "Test")
public class PinpointOdometryTest extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ---------------- PINPOINT MOUNTING ----------------
    // +X = left of robot center, -X = right
    // +Y = forward of robot center, -Y = backward
    private static final double PINPOINT_OFFSET_DX_IN = -6.5;
    private static final double PINPOINT_OFFSET_DY_IN = -5.75;

    // ---------------- LIMELIGHT ----------------
    private static final int APRILTAG_PIPELINE = 2;
    private static final long MAX_STALENESS_MS = 200;
    private static final long SEED_TIMEOUT_MS = 1500;

    // ---------------- DASHBOARD ----------------
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double HALF_FIELD_IN = FIELD_SIZE_IN / 2.0;

    private static final double ROBOT_LENGTH_IN = 18.0;
    private static final double ROBOT_WIDTH_IN  = 16.0;
    private static final double ARROW_LEN_IN    = 10.0;

    // ---------------- DRIVE TUNING ----------------
    private static final double DEADBAND_FWD = 0.12;
    private static final double DEADBAND_STRAFE = 0.05;
    private static final double DEADBAND_ROT = 0.05;

    // Snap-to-heading tuning
    private static final double SNAP_KP = 0.012;       // power per degree of error (start here)
    private static final double SNAP_MAX = 0.65;       // cap rotate power
    private static final double SNAP_MIN = 0.08;       // minimum rotate to overcome stiction
    private static final double SNAP_DONE_DEG = 2.0;   // consider "at target" within this many degrees

    // Snap state
    private boolean snapping = false;
    private double snapTargetDeg = 0.0;

    @Override
    public void runOpMode() {

        // ---------- HARDWARE ----------
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Motor directions (typical mecanum)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake (reduce glide)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        // Reset before seeding
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Ready. Press START.");
        telemetry.addLine("Dpad snap: Right=90  Left=-90  Up=180  Down=0");
        telemetry.update();
        waitForStart();

        // ---------- SEED ONCE ----------
        Pose2D initialPose = waitForFreshLimelightPose(SEED_TIMEOUT_MS);
        if (initialPose != null) {
            pinpoint.setPosition(initialPose);
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();

        while (opModeIsActive()) {

            // ---------- UPDATE ODOM ----------
            pinpoint.update();

            // Combined heading (seeded from LL once, then odom integrated)
            double headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

            // ---------- DPAD SNAP TARGET ----------
            if (gamepad1.dpad_right) { snapping = true; snapTargetDeg =  90.0; }
            else if (gamepad1.dpad_left) { snapping = true; snapTargetDeg = -90.0; }
            else if (gamepad1.dpad_up) { snapping = true; snapTargetDeg = 180.0; }
            else if (gamepad1.dpad_down) { snapping = true; snapTargetDeg =   0.0; }

            // ---------- DRIVETRAIN INPUT ----------
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double rotate  =  gamepad1.right_stick_x;

            // Deadbands
            if (Math.abs(forward) < DEADBAND_FWD) forward = 0;
            if (Math.abs(strafe)  < DEADBAND_STRAFE) strafe = 0;
            if (Math.abs(rotate)  < DEADBAND_ROT) rotate = 0;

            // If snapping, override rotate with shortest-path controller
            if (snapping) {
                double errDeg = wrapDeg(snapTargetDeg - headingDeg); // shortest path error in [-180,180)

                if (Math.abs(errDeg) <= SNAP_DONE_DEG) {
                    // Close enough → stop snapping (let driver rotate again)
                    snapping = false;
                    rotate = 0;
                } else {
                    double cmd = SNAP_KP * errDeg;

                    // Clamp
                    cmd = clamp(cmd, -SNAP_MAX, SNAP_MAX);

                    // Minimum power to overcome friction (keep sign)
                    if (Math.abs(cmd) < SNAP_MIN) cmd = Math.copySign(SNAP_MIN, cmd);

                    rotate = cmd;
                }
            }

            // ---------- MECANUM MIX ----------
            double fl = forward + strafe + rotate;
            double fr = forward - strafe - rotate;
            double bl = forward - strafe + rotate;
            double br = forward + strafe - rotate;

            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(fr),
                                    Math.max(Math.abs(bl), Math.abs(br)))));

            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(bl / max);
            backRight.setPower(br / max);

            // ---------- POSE ----------
            double xIn = pinpoint.getPosX(DistanceUnit.INCH);
            double yIn = pinpoint.getPosY(DistanceUnit.INCH);

            // ---------- DASHBOARD ----------
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();
            field.strokeRect(-HALF_FIELD_IN, -HALF_FIELD_IN, FIELD_SIZE_IN, FIELD_SIZE_IN);
            drawRobotRect(field, xIn, yIn, headingDeg);
            drawArrow(field, xIn, yIn, headingDeg);
            packet.put("heading_deg", headingDeg);
            packet.put("snapping", snapping);
            packet.put("snap_target_deg", snapTargetDeg);
            dashboard.sendTelemetryPacket(packet);

            // ---------- TELEMETRY ----------
            telemetry.addData("X (in)", "%.2f", xIn);
            telemetry.addData("Y (in)", "%.2f", yIn);
            telemetry.addData("Heading (deg)", "%.1f", headingDeg);
            telemetry.addData("Snap", snapping ? ("ON -> " + snapTargetDeg) : "OFF");
            telemetry.update();
        }

        limelight.stop();
    }

    // ---------------- HELPERS ----------------

    private Pose2D waitForFreshLimelightPose(long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeoutMs) {
            Pose2D p = getFreshLimelightPoseOnce();
            if (p != null) return p;
            sleep(10);
        }
        return null;
    }

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

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Wrap angle to [-180, 180)
    private static double wrapDeg(double deg) {
        deg = ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return deg;
    }

    private void drawRobotRect(Canvas c, double cx, double cy, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        double hl = ROBOT_LENGTH_IN / 2.0;
        double hw = ROBOT_WIDTH_IN / 2.0;

        double fx = Math.cos(h), fy = Math.sin(h);
        double lx = -Math.sin(h), ly = Math.cos(h);

        double[] xs = {
                cx + fx*hl + lx*hw,
                cx + fx*hl - lx*hw,
                cx - fx*hl - lx*hw,
                cx - fx*hl + lx*hw,
                cx + fx*hl + lx*hw
        };

        double[] ys = {
                cy + fy*hl + ly*hw,
                cy + fy*hl - ly*hw,
                cy - fy*hl - ly*hw,
                cy - fy*hl + ly*hw,
                cy + fy*hl + ly*hw
        };

        c.strokePolyline(xs, ys);
    }

    private void drawArrow(Canvas c, double x, double y, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        c.strokeLine(x, y,
                x + ARROW_LEN_IN * Math.cos(h),
                y + ARROW_LEN_IN * Math.sin(h));
    }
}