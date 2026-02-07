package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private DcMotor shootMotor;

    // Continuous servos
    private CRServo rightTransfer, leftTransfer, spindexServo;

    // Positional servo
    private Servo flywheelMover;

    // ---------------- PINPOINT MOUNTING ----------------
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

    // ---------------- DRIVE ----------------
    private static final double DEADBAND_FWD = 0.12;
    private static final double DEADBAND_STRAFE = 0.05;
    private static final double DEADBAND_ROT = 0.05;

    // ---------------- TOGGLES ----------------
    private boolean shooterOn = false;
    private boolean transferOn = false;

    private boolean lastRT = false;
    private boolean lastLT = false;

    private boolean lastLB = false;
    private boolean lastRB = false;

    private static final double TRIGGER_PRESSED = 0.60;

    // ---------------- FLYWHEEL MOVER POSITIONS ----------------
    // PLACEHOLDERS – tune later
    private static final double FLYWHEEL_MOVER_POS_A = 0.20;
    private static final double FLYWHEEL_MOVER_POS_B = 0.80;

    // ============================================================
    //                 SPINDEX (TIMED PULSE ON X)
    // ============================================================
    private static final double SPIN_POWER = 0.25;
    private static final double STOP_POWER = 0.0;
    private static final double[] RUN_TIMES = {0.20, 0.24, 0.23, 0.26, 0.25};

    private boolean spindexSpinning = false;
    private final ElapsedTime spindexTimer = new ElapsedTime();
    private int spindexRunIndex = 0;
    private boolean xArmed = true;

    private DcMotor intakeMotor;
    private boolean intakeOn = false;
    private boolean lastB = false;

    private static final double SHOOTER_POWER = 0.9; // change this

    // ============================================================
    //                 FIELD-CENTRIC TUNING
    // ============================================================
    // Your desired field convention:
    //   stick forward -> toward field -X
    //   stick right   -> toward field +Y
    //   stick left    -> toward field -Y
    //
    // If it feels mirrored/rotated, flip these:
    private static final int HEADING_SIGN = -1;            // flip to +1 if needed
    private static final double HEADING_OFFSET_DEG = 180.0; // change to 0.0 if needed

    @Override
    public void runOpMode() {

        // ---------- HARDWARE ----------
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        shootMotor = hardwareMap.get(DcMotor.class, "shootMotor");

        // CRServos
        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");
        leftTransfer  = hardwareMap.get(CRServo.class, "leftTransfer");
        spindexServo  = hardwareMap.get(CRServo.class, "spindexServo");

        // Positional servo
        flywheelMover = hardwareMap.get(Servo.class, "flywheelMover");

        // Motor directions (typical mecanum)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ✅ BRAKE prevents drift when power=0
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        // Shooter: brake when off
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setPower(0);

        // Stop CRServos initially
        rightTransfer.setPower(0);
        leftTransfer.setPower(0);
        spindexServo.setPower(0);

        // ---------- PINPOINT ----------
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
        telemetry.addLine("RT click = toggle shootMotor");
        telemetry.addLine("LT click = toggle left/right transfers");
        telemetry.addLine("LB/RB click = flywheelMover pos A/B");
        telemetry.addLine("B click = toggle intakeMotor");
        telemetry.addLine("X press = timed spindex pulse");
        telemetry.update();

        waitForStart();

        // ---------- SEED ONCE ----------
        Pose2D initialPose = waitForFreshLimelightPose(SEED_TIMEOUT_MS);
        boolean seeded = false;
        if (initialPose != null) {
            pinpoint.setPosition(initialPose);
            seeded = true;
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();

        while (opModeIsActive()) {

            // ---------- TRIGGER TOGGLES ----------
            boolean rt = gamepad1.right_trigger > TRIGGER_PRESSED;
            boolean lt = gamepad1.left_trigger  > TRIGGER_PRESSED;

            boolean rtClicked = rt && !lastRT;
            boolean ltClicked = lt && !lastLT;

            lastRT = rt;
            lastLT = lt;

            if (rtClicked) {
                shooterOn = !shooterOn;
                shootMotor.setPower(shooterOn ? SHOOTER_POWER : 0.0);
            }

            if (ltClicked) {
                transferOn = !transferOn;
                if (transferOn) {
                    rightTransfer.setPower(+1.0);
                    leftTransfer.setPower(-1.0);
                } else {
                    rightTransfer.setPower(0.0);
                    leftTransfer.setPower(0.0);
                }
            }

            // ---------- FLYWHEEL MOVER (BUMPERS) ----------
            boolean lbClicked = gamepad1.left_bumper && !lastLB;
            boolean rbClicked = gamepad1.right_bumper && !lastRB;

            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;

            if (lbClicked) flywheelMover.setPosition(FLYWHEEL_MOVER_POS_A);
            if (rbClicked) flywheelMover.setPosition(FLYWHEEL_MOVER_POS_B);

            // ---------- INTAKE TOGGLE (B) ----------
            boolean bPressed = gamepad1.b;
            boolean bClicked = bPressed && !lastB;
            lastB = bPressed;

            if (bClicked) {
                intakeOn = !intakeOn;
                intakeMotor.setPower(intakeOn ? 1.0 : 0.0);
            }

            // ============================================================
            //             SPINDEX TIMED PULSE (X BUTTON)
            // ============================================================
            boolean xPressed = gamepad1.x;

            // re-arm when released
            if (!xPressed) xArmed = true;

            // start pulse on new press (armed) if not already spinning
            if (xPressed && xArmed && !spindexSpinning) {
                spindexSpinning = true;
                spindexTimer.reset();
                xArmed = false;
            }

            if (spindexSpinning) {
                double runTime = RUN_TIMES[spindexRunIndex];
                if (spindexTimer.seconds() >= runTime) {
                    spindexServo.setPower(STOP_POWER);
                    spindexSpinning = false;
                    spindexRunIndex = (spindexRunIndex + 1) % RUN_TIMES.length;
                } else {
                    spindexServo.setPower(SPIN_POWER);
                }
            } else {
                spindexServo.setPower(STOP_POWER);
            }
            // ============================================================

            // ---------- ODOM ----------
            pinpoint.update();
            double xIn = pinpoint.getPosX(DistanceUnit.INCH);
            double yIn = pinpoint.getPosY(DistanceUnit.INCH);
            double heading = pinpoint.getHeading(AngleUnit.DEGREES);

            // ============================================================
            //                 FIELD-CENTRIC DRIVE (ONLY DRIVE CHANGE)
            // ============================================================
            double stickForward = -gamepad1.left_stick_y; // push forward => +
            double stickStrafe  =  gamepad1.left_stick_x; // push right   => +
            double rotate       =  gamepad1.right_stick_x;

            if (Math.abs(stickForward) < DEADBAND_FWD) stickForward = 0;
            if (Math.abs(stickStrafe)  < DEADBAND_STRAFE) stickStrafe = 0;
            if (Math.abs(rotate)       < DEADBAND_ROT) rotate = 0;

            // Rotate field vector into robot frame using Pinpoint heading
            double h = Math.toRadians(HEADING_SIGN * (heading + HEADING_OFFSET_DEG));
            double cos = Math.cos(h);
            double sin = Math.sin(h);

            // Field vector: x = strafe, y = forward
            double xField = stickStrafe;
            double yField = stickForward;

            // Field -> Robot
            double xRobot = xField * cos - yField * sin;   // robot strafe
            double yRobot = xField * sin + yField * cos;   // robot forward

            double forward = yRobot;
            double strafe  = xRobot;

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
            // ============================================================

            // ---------- DASHBOARD ----------
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();
            field.strokeRect(-HALF_FIELD_IN, -HALF_FIELD_IN, FIELD_SIZE_IN, FIELD_SIZE_IN);
            drawRobotRect(field, xIn, yIn, heading);
            drawArrow(field, xIn, yIn, heading);

            packet.put("seeded", seeded);
            packet.put("x_in", xIn);
            packet.put("y_in", yIn);
            packet.put("heading_deg", heading);
            packet.put("shooterOn", shooterOn);
            packet.put("transferOn", transferOn);
            packet.put("spindexRunning", spindexSpinning);
            packet.put("spindexRunTime", RUN_TIMES[spindexRunIndex]);

            dashboard.sendTelemetryPacket(packet);

            // ---------- TELEMETRY ----------
            telemetry.addData("BRAKE", "ENABLED");
            telemetry.addData("X (in)", "%.2f", xIn);
            telemetry.addData("Y (in)", "%.2f", yIn);
            telemetry.addData("Heading (deg)", "%.1f", heading);
            telemetry.addData("Seeded", seeded ? "YES" : "NO");
            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Transfer", transferOn ? "ON" : "OFF");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Spindexer Running", spindexSpinning);
            telemetry.addData("Spindex Run Time (s)", "%.2f", RUN_TIMES[spindexRunIndex]);
            telemetry.addData("FlywheelMover Pos", "%.2f", flywheelMover.getPosition());
            telemetry.update();
        }

        // stop everything on exit
        shootMotor.setPower(0);
        rightTransfer.setPower(0);
        leftTransfer.setPower(0);
        spindexServo.setPower(0);
        intakeMotor.setPower(0);

        limelight.stop();
    }

    // ---------------- LIMELIGHT HELPERS ----------------
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

    // ---------------- DASHBOARD DRAWING ----------------
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