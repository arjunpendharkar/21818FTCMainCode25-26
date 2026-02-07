package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name="Turret Aim Blue (Pinpoint+LL)", group="Test")
public class TurretAimBlueTest extends LinearOpMode {

    // -------- Hardware names --------
    private static final String PINPOINT_NAME = "pinpoint";
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String TURRET_SERVO_NAME = "turretServo";

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private Servo turretServo;

    // -------- Limelight --------
    private static final int APRILTAG_PIPELINE = 2;
    private static final long MAX_STALENESS_MS = 200;
    private static final long SEED_TIMEOUT_MS = 1500;

    // -------- Goal (from your ftc2025DECODE.fmap) --------
    // Map contained fiducials:
    //  id 20 at (-1.4827, -1.4133) meters
    //  id 24 at (-1.4827, +1.4133) meters
    // We'll aim at BLUE = tag 24 for now.
    private static final double BLUE_GOAL_X_IN = DistanceUnit.METER.toInches(-1.4827);
    private static final double BLUE_GOAL_Y_IN = DistanceUnit.METER.toInches( 1.4133);

    // -------- Turret math --------
    // Your turret gearing: servo gear smaller, turret gear bigger = 5.4:1 reduction
    // turretDeg = servoDeg / 5.4  => servoDeg = turretDeg * 5.4
    private static final double TURRET_GEAR_RATIO = 5.4;

    // goBILDA 5-turn servo range is up to ~1800° over position 0..1 (when using wide PWM)
    private static final double SERVO_TOTAL_DEG = 1800.0;

    // You MUST tune these:
    // "When turret is pointing perfectly at robot-forward (turretRelDeg=0), what servo position should it be?"
    private static final double SERVO_ZERO_POS = 0.50;   // placeholder
    private static final double SERVO_ZERO_DEG = 0.0;    // keep 0 unless you want an extra offset

    // Safety clamp so we don't slam endpoints while tuning
    private static final double SERVO_MIN = 0.02;
    private static final double SERVO_MAX = 0.98;

    @Override
    public void runOpMode() {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        turretServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);

        // Limelight setup
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        // Reset pinpoint before seeding
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Turret Aim Blue (Pinpoint+LL)");
        telemetry.addLine("Aims turret at BLUE goal using robot pose + goal coords.");
        telemetry.addLine("NOTE: servo getPosition() is NOT real feedback (just last command).");
        telemetry.update();

        waitForStart();

        // Seed Pinpoint once from Limelight
        Pose2D initialPose = waitForFreshLimelightPose(SEED_TIMEOUT_MS);
        boolean seeded = false;
        if (initialPose != null) {
            pinpoint.setPosition(initialPose);
            seeded = true;
        }

        while (opModeIsActive()) {

            pinpoint.update();

            // Robot pose in field coords (inches)
            double rx = pinpoint.getPosX(DistanceUnit.INCH);
            double ry = pinpoint.getPosY(DistanceUnit.INCH);

            // Heading convention matches your field headings:
            // 0 = field back (+X), 90 = field right (+Y), 180 = field forward (-X), -90 = field left (-Y)
            double robotHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

            // Vector to blue goal
            double dx = BLUE_GOAL_X_IN - rx;
            double dy = BLUE_GOAL_Y_IN - ry;

            // Absolute field bearing to the goal (same convention as your headings)
            double bearingDeg = Math.toDegrees(Math.atan2(dy, dx));

            // Turret needs to rotate relative to robot so turret points at the goal
            double turretRelDeg = wrapDeg(bearingDeg - robotHeadingDeg);

            // Convert turret angle -> servo angle through gear ratio
            double servoDeg = (turretRelDeg * TURRET_GEAR_RATIO) + SERVO_ZERO_DEG;

            // Convert servo degrees to servo position (0..1), centered around SERVO_ZERO_POS
            double servoPos = SERVO_ZERO_POS + (servoDeg / SERVO_TOTAL_DEG);

            // Wrap position so turret can go either way (optional). If you don’t want wrapping, remove this.
            servoPos = wrap01(servoPos);

            // Clamp for safety while tuning
            servoPos = clamp(servoPos, SERVO_MIN, SERVO_MAX);

            // Command turret
            turretServo.setPosition(servoPos);

            telemetry.addData("Seeded", seeded ? "YES" : "NO");
            telemetry.addData("Robot X (in)", "%.2f", rx);
            telemetry.addData("Robot Y (in)", "%.2f", ry);
            telemetry.addData("Robot Heading (deg)", "%.1f", robotHeadingDeg);

            telemetry.addData("Blue Goal X (in)", "%.2f", BLUE_GOAL_X_IN);
            telemetry.addData("Blue Goal Y (in)", "%.2f", BLUE_GOAL_Y_IN);

            telemetry.addData("Bearing to Goal (deg)", "%.1f", bearingDeg);
            telemetry.addData("Turret Rel Target (deg)", "%.1f", turretRelDeg);
            telemetry.addData("Servo Target Pos", "%.4f", servoPos);
            telemetry.addData("Servo getPosition()", "%.4f", turretServo.getPosition());

            telemetry.update();
        }

        limelight.stop();
    }

    // ---------------- Limelight seeding helpers ----------------

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

    // ---------------- Math helpers ----------------

    // Wrap degrees to (-180..180]
    private static double wrapDeg(double deg) {
        deg = ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return deg;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Wrap a value into [0,1)
    private static double wrap01(double p) {
        p = p % 1.0;
        if (p < 0) p += 1.0;
        return p;
    }
}