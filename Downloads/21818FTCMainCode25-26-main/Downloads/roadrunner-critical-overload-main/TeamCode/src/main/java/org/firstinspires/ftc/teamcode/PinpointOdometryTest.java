package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "LL Seed Once -> Pinpoint Odom + Dashboard", group = "Test")
public class PinpointOdometryTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    // Limelight AprilTag pipeline + freshness thresholds
    private static final int APRILTAG_PIPELINE = 2;
    private static final long MAX_STALENESS_MS = 200;
    private static final long SEED_TIMEOUT_MS = 1500; // wait up to this long at start for initial pose

    // Dashboard drawing (inches)
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double HALF_FIELD_IN = FIELD_SIZE_IN / 2.0;

    // Robot: 18x16 rectangle, front is the longer side (18)
    private static final double ROBOT_LENGTH_IN = 18.0; // front-to-back
    private static final double ROBOT_WIDTH_IN  = 16.0; // left-to-right
    private static final double ARROW_LEN_IN    = 10.0;

    // Dots
    private static final double DOT_R_IN = 1.5; // radius in inches

    @Override
    public void runOpMode() {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ---- Pinpoint configuration (your setup) ----
        pinpoint.setOffsets(
                OdometryConstants.PINPOINT_OFFSET_DX_INCHES,
                OdometryConstants.PINPOINT_OFFSET_DY_INCHES,
                DistanceUnit.INCH
        );
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED, // X
                GoBildaPinpointDriver.EncoderDirection.FORWARD   // Y
        );

        // Start Limelight
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        // Reset Pinpoint to a known state BEFORE seeding
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Ready. Press START.");
        telemetry.addLine("After START: seed once from Limelight (if fresh), then odometry-only.");
        telemetry.update();

        waitForStart();

        // ---- SEED ONCE at start from Limelight ----
        Pose2D initialPose = waitForFreshLimelightPose(SEED_TIMEOUT_MS);
        boolean seeded = false;

        if (initialPose != null) {
            pinpoint.setPosition(initialPose);
            seeded = true;
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();

        while (opModeIsActive()) {

            // REQUIRED: update pinpoint every loop
            pinpoint.update();

            // ---- Current Pinpoint pose (odometry-integrated) ----
            double ppXIn = pinpoint.getPosX(DistanceUnit.INCH);
            double ppYIn = pinpoint.getPosY(DistanceUnit.INCH);
            double ppXM  = pinpoint.getPosX(DistanceUnit.METER);
            double ppYM  = pinpoint.getPosY(DistanceUnit.METER);
            double ppHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

            // ---- Current Limelight pose for display (does NOT override pinpoint) ----
            Pose2D llPoseNow = getFreshLimelightPoseOnce();
            boolean llFreshNow = (llPoseNow != null);

            // ---------- FTC DASHBOARD FIELD ----------
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            // field border + axes (origin at center)
            field.strokeRect(-HALF_FIELD_IN, -HALF_FIELD_IN, FIELD_SIZE_IN, FIELD_SIZE_IN);
            field.strokeLine(-HALF_FIELD_IN, 0, HALF_FIELD_IN, 0);
            field.strokeLine(0, -HALF_FIELD_IN, 0, HALF_FIELD_IN);

            // robot (Pinpoint current)
            drawRobotRect(field, ppXIn, ppYIn, ppHeadingDeg, ROBOT_LENGTH_IN, ROBOT_WIDTH_IN);
            drawArrow(field, ppXIn, ppYIn, ppHeadingDeg, ARROW_LEN_IN);

            // initial pose dot (seed)
            if (initialPose != null) {
                double ix = initialPose.getX(DistanceUnit.INCH);
                double iy = initialPose.getY(DistanceUnit.INCH);
                drawDot(field, ix, iy, DOT_R_IN);
                packet.put("initial_x_in", ix);
                packet.put("initial_y_in", iy);
            } else {
                packet.put("initial_pose", "null");
            }

            // limelight dot (current, if fresh)
            if (llFreshNow) {
                double lx = llPoseNow.getX(DistanceUnit.INCH);
                double ly = llPoseNow.getY(DistanceUnit.INCH);
                drawDot(field, lx, ly, DOT_R_IN);
                packet.put("ll_x_in", lx);
                packet.put("ll_y_in", ly);
            } else {
                packet.put("ll_fresh", false);
            }

            packet.put("pp_x_in", ppXIn);
            packet.put("pp_y_in", ppYIn);
            packet.put("pp_heading_deg", ppHeadingDeg);
            packet.put("seeded", seeded);

            dashboard.sendTelemetryPacket(packet);
            // ---------- END DASHBOARD ----------

            // ---------- DRIVER STATION TELEMETRY ----------
            telemetry.addLine("=== INITIAL POSE (seed once at START) ===");
            telemetry.addData("Seeded from LL", seeded ? "YES" : "NO");
            if (initialPose != null) {
                telemetry.addData("Init X (in)", "%.2f", initialPose.getX(DistanceUnit.INCH));
                telemetry.addData("Init Y (in)", "%.2f", initialPose.getY(DistanceUnit.INCH));
                telemetry.addData("Init X (m)", "%.3f", initialPose.getX(DistanceUnit.METER));
                telemetry.addData("Init Y (m)", "%.3f", initialPose.getY(DistanceUnit.METER));
                telemetry.addData("Init Heading (deg)", "%.1f", initialPose.getHeading(AngleUnit.DEGREES));
            } else {
                telemetry.addLine("Initial Pose: n/a (no fresh LL pose at start)");
            }

            telemetry.addLine("");
            telemetry.addLine("=== LIMELIGHT (current, display only) ===");
            telemetry.addData("LL pose fresh", llFreshNow ? "YES" : "NO");
            if (llFreshNow) {
                telemetry.addData("LL X (in)", "%.2f", llPoseNow.getX(DistanceUnit.INCH));
                telemetry.addData("LL Y (in)", "%.2f", llPoseNow.getY(DistanceUnit.INCH));
                telemetry.addData("LL X (m)", "%.3f", llPoseNow.getX(DistanceUnit.METER));
                telemetry.addData("LL Y (m)", "%.3f", llPoseNow.getY(DistanceUnit.METER));
                telemetry.addData("LL Heading (deg)", "%.1f", llPoseNow.getHeading(AngleUnit.DEGREES));
            } else {
                telemetry.addLine("LL Pose: n/a");
            }

            telemetry.addLine("");
            telemetry.addLine("=== PINPOINT (current, odometry) ===");
            telemetry.addData("X (in)", "%.2f", ppXIn);
            telemetry.addData("Y (in)", "%.2f", ppYIn);
            telemetry.addData("X (m)", "%.3f", ppXM);
            telemetry.addData("Y (m)", "%.3f", ppYM);
            telemetry.addData("Heading (deg)", "%.1f", ppHeadingDeg);

            telemetry.update();
            // ---------- END TELEMETRY ----------
        }

        limelight.stop();
    }

    /**
     * Wait up to timeoutMs for a fresh Limelight pose (blocking).
     * Returns null if none arrives in time.
     */
    private Pose2D waitForFreshLimelightPose(long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {
            Pose2D p = getFreshLimelightPoseOnce();
            if (p != null) return p;
            sleep(10);
        }
        return null;
    }

    /**
     * Non-blocking: returns a Pose2D only if Limelight has a fresh pose right now.
     */
    private Pose2D getFreshLimelightPoseOnce() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) return null;

        long stalenessMs = llResult.getStaleness();
        if (stalenessMs < 0 || stalenessMs > MAX_STALENESS_MS) return null;

        Pose3D botpose = llResult.getBotpose();
        if (botpose == null) return null;

        return toPose2D(botpose);
    }

    /**
     * Convert Limelight botpose (meters + degrees) into Pose2D in inches.
     */
    private Pose2D toPose2D(Pose3D botpose) {
        Position p = botpose.getPosition();
        double xIn = DistanceUnit.METER.toInches(p.x);
        double yIn = DistanceUnit.METER.toInches(p.y);
        double headingDeg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg);
    }

    // ---------------- Dashboard drawing helpers ----------------

    private void drawDot(Canvas c, double x, double y, double r) {
        // strokeCircle exists on dashboard Canvas
        c.strokeCircle(x, y, r);
    }

    private void drawRobotRect(Canvas c, double cx, double cy, double headingDeg,
                               double lengthIn, double widthIn) {

        double h = Math.toRadians(headingDeg);

        double hl = lengthIn / 2.0;
        double hw = widthIn  / 2.0;

        // forward unit vector (heading)
        double fx = Math.cos(h);
        double fy = Math.sin(h);

        // left unit vector
        double lx = -Math.sin(h);
        double ly =  Math.cos(h);

        // corners: front-left, front-right, back-right, back-left
        double flx = cx + fx * hl + lx * hw;
        double fly = cy + fy * hl + ly * hw;

        double frx = cx + fx * hl - lx * hw;
        double fry = cy + fy * hl - ly * hw;

        double brx = cx - fx * hl - lx * hw;
        double bry = cy - fy * hl - ly * hw;

        double blx = cx - fx * hl + lx * hw;
        double bly = cy - fy * hl + ly * hw;

        c.strokePolyline(
                new double[]{flx, frx, brx, blx, flx},
                new double[]{fly, fry, bry, bly, fly}
        );
    }

    private void drawArrow(Canvas c, double x, double y, double headingDeg, double len) {
        double h = Math.toRadians(headingDeg);

        double x2 = x + len * Math.cos(h);
        double y2 = y + len * Math.sin(h);

        c.strokeLine(x, y, x2, y2);

        // arrow head
        double headLen = 4.0;
        double headAngle = Math.toRadians(25);

        double lx = x2 - headLen * Math.cos(h - headAngle);
        double ly = y2 - headLen * Math.sin(h - headAngle);
        double rx = x2 - headLen * Math.cos(h + headAngle);
        double ry = y2 - headLen * Math.sin(h + headAngle);

        c.strokeLine(x2, y2, lx, ly);
        c.strokeLine(x2, y2, rx, ry);
    }
}