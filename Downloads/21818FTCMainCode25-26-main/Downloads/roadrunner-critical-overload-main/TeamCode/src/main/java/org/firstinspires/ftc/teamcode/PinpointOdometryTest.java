package org.firstinspires.ftc.teamcode;

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

@TeleOp(name = "Pinpoint Odometry Test", group = "Test")
public class PinpointOdometryTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    // Limelight AprilTag (pipeline 2) pose validity thresholds
    private static final int APRILTAG_PIPELINE = 2;
    private static final long MAX_STALENESS_MS = 200;

    // Mode flags
    private boolean seededFromLimelight = false;          // auto-once seed guard
    private boolean continuousCorrectionEnabled = true;   // apply pose whenever valid

    // Button edges
    private boolean lastBack = false;

    @Override
    public void runOpMode() {

        // Get hardware
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Reset position + IMU so we start at (0,0,0)
        pinpoint.resetPosAndIMU();

        // Configure Limelight: switch to AprilTag pipeline (2) and start polling
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Ready. Spin a pod; waiting for Limelight AprilTag pose.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // REQUIRED: update Pinpoint every loop
            pinpoint.update();

            // Read Limelight botpose from AprilTag pipeline
            LLResult llResult = limelight.getLatestResult();
            Pose2D limelightPose = null;
            boolean llPoseFresh = false;

            if (llResult != null && llResult.isValid()) {
                long stalenessMs = llResult.getStaleness();
                Pose3D botpose = llResult.getBotpose();
                if (botpose != null && stalenessMs >= 0 && stalenessMs <= MAX_STALENESS_MS) {
                    limelightPose = toPose2D(botpose);
                    llPoseFresh = true;
                }
            }

            // Auto-once seed: first fresh pose initializes Pinpoint
            if (!seededFromLimelight && llPoseFresh && limelightPose != null) {
                pinpoint.setPosition(limelightPose);
                seededFromLimelight = true;
            }

            // Continuous correction (optional)
            if (continuousCorrectionEnabled && llPoseFresh && limelightPose != null) {
                pinpoint.setPosition(limelightPose);
            }

            // Manual resync on back button edge
            boolean backPressed = gamepad1.back;
            boolean backTapped = backPressed && !lastBack;
            lastBack = backPressed;
            if (backTapped && limelightPose != null) {
                pinpoint.setPosition(limelightPose);
                seededFromLimelight = true; // ensure flag is set even if first time
            }

            // Read position and heading WITH UNITS
            double x = pinpoint.getPosX(DistanceUnit.INCH);
            double y = pinpoint.getPosY(DistanceUnit.INCH);
            double heading = pinpoint.getHeading(AngleUnit.DEGREES);

            telemetry.addData("X (in)", "%.2f", x);
            telemetry.addData("Y (in)", "%.2f", y);
            telemetry.addData("Heading (deg)", "%.1f", heading);
            telemetry.addData("LL pose fresh", llPoseFresh ? "YES" : "NO");
            if (limelightPose != null) {
                telemetry.addData("LL Pose (in)", "x=%.1f y=%.1f hdg=%.1f",
                        limelightPose.getX(DistanceUnit.INCH),
                        limelightPose.getY(DistanceUnit.INCH),
                        limelightPose.getHeading(AngleUnit.DEGREES));
            } else {
                telemetry.addLine("LL Pose (in): n/a");
            }
            telemetry.addData("Seeded from LL", seededFromLimelight ? "YES" : "NO");
            telemetry.addData("Continuous LL assist", continuousCorrectionEnabled ? "ON" : "OFF");
            telemetry.update();
        }

        limelight.stop();
    }

    /**
     * Convert Limelight botpose (meters + degrees) into Pinpoint-compatible Pose2D in inches.
     */
    private Pose2D toPose2D(Pose3D botpose) {
        Position p = botpose.getPosition();
        double xIn = DistanceUnit.METER.toInches(p.x);
        double yIn = DistanceUnit.METER.toInches(p.y);
        double headingDeg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg);
    }
}
