package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Sample OpMode that seeds odometry from Limelight MegaTag 1 (AprilTag).
 * - Grabs tx/ty/RZ from MegaTag 1 at start.
 * - Converts those readings into a field pose (x, y, heading in radians).
 * - Initializes Pinpoint odometry with that pose and keeps updating.
 * - Maintains a Pose2d currentPose (field inches, heading radians).
 *
 * Notes:
 *  - Set the Limelight pipeline so only MegaTag (AprilTag) is running.
 *  - Update TAG_* and camera calibration constants to match your field and robot.
 */
@TeleOp(name = "Limelight Seeded Odometry", group = "Sensor")
public class LimelightSeededOdometry extends LinearOpMode {

    // ===== User calibration/constants (update for your robot/field) =====
    private static final int APRILTAG_PIPELINE = 2;          // Limelight pipeline index for AprilTag / MegaTag
    private static final int TARGET_FIDUCIAL_ID = 1;         // MegaTag 1 only
    private static final double TAG_X_IN = 0.0;              // Field X of MegaTag 1 (inches, field coord)
    private static final double TAG_Y_IN = 0.0;              // Field Y of MegaTag 1 (inches, field coord)
    private static final double TAG_YAW_DEG = 0.0;           // Field-facing yaw of MegaTag 1 (deg, CCW from +X)

    // Camera mounting (used for optional distance calculation)
    private static final double CAMERA_HEIGHT_M = 0.26;      // meters
    private static final double TAG_HEIGHT_M = 0.14;         // meters (center of tag)
    private static final double CAMERA_PITCH_DEG = 20.0;     // deg up from horizontal (+ looks upward)

    // ===== Hardware =====
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    // Exposed field pose in inches / radians
    public Pose2d currentPose = new Pose2d(0, 0, 0);

    private boolean seededFromLimelight = false;
    private boolean lastBack = false; // for manual re-seed

    @Override
    public void runOpMode() {

        // Map hardware
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure Pinpoint using measured offsets from OdometryConstants
        configurePinpoint();

        // Configure Limelight for MegaTag (AprilTag) and start polling
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Init: waiting for MegaTag 1...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Always update Pinpoint; it accumulates odometry internally
            pinpoint.update();

            // Read Limelight once per loop for initial seed + manual re-seed
            TagObservation obs = readMegaTag1(limelight.getLatestResult());

            // Try to seed once from MegaTag 1 (auto only the first time)
            if (!seededFromLimelight && obs != null) {
                seedPinpointFromObservation(obs);
                telemetry.addLine("Seeded odometry from MegaTag 1");
            }

            // Manual re-seed on gamepad1.back edge (only if tag is seen)
            boolean backPressed = gamepad1.back;
            boolean backTapped = backPressed && !lastBack;
            lastBack = backPressed;
            if (backTapped && obs != null) {
                seedPinpointFromObservation(obs);
                telemetry.addLine("Manual re-seed from MegaTag 1");
            }

            // Read odometry and maintain normalized heading
            double xIn = pinpoint.getPosX(DistanceUnit.INCH);
            double yIn = pinpoint.getPosY(DistanceUnit.INCH);
            double headingRad = wrapToPi(Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES)));
            currentPose = new Pose2d(xIn, yIn, headingRad);

            // Telemetry for debugging
            telemetry.addData("Pose (in/rad)", "x=%.1f y=%.1f h=%.2f", currentPose.x, currentPose.y, currentPose.heading);
            telemetry.addData("Seeded", seededFromLimelight);

            if (obs != null) {
                telemetry.addData("tx (deg)", "%.2f", obs.txDeg);
                telemetry.addData("ty (deg)", "%.2f", obs.tyDeg);
                telemetry.addData("RZ rel tag (deg)", "%.2f", obs.rzDeg);
                telemetry.addData("Tag field (in)", "x=%.1f y=%.1f yaw=%.1f", obs.tagXIn, obs.tagYIn, obs.tagYawDeg);
                telemetry.addData("Est dist (in)", "%.1f", DistanceUnit.METER.toInches(estimateDistanceMeters(obs.tyDeg)));
            } else {
                telemetry.addLine("MegaTag 1 not visible");
            }
            telemetry.update();
        }

        limelight.stop();
    }

    /**
     * Write a single MegaTag observation into Pinpoint as the current pose.
     */
    private void seedPinpointFromObservation(TagObservation obs) {
        Pose2d startPose = limelightToFieldPose(obs);
        Pose2D seed = new Pose2D(
                DistanceUnit.INCH, startPose.x,
                startPose.y, AngleUnit.RADIANS,
                startPose.heading
        );
        pinpoint.setPosition(seed);
        seededFromLimelight = true;
    }

    /**
     * Configure Pinpoint offsets and encoder settings using OdometryConstants.
     */
    private void configurePinpoint() {
        // Odometry pod offsets relative to the robot tracking point (inches)
        pinpoint.setOffsets(
                OdometryConstants.PINPOINT_OFFSET_DX_INCHES,
                OdometryConstants.PINPOINT_OFFSET_DY_INCHES,
                DistanceUnit.INCH);

        // Use goBILDA 4-bar pods resolution; adjust if you use a different encoder
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Ensure encoder directions increase forward (X pod) and left (Y pod)
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset pose and recalibrate IMU after configuration
        pinpoint.resetPosAndIMU();
    }

    /**
    * Reads MegaTag 1 data (tx/ty + derived RZ) from the latest Limelight result.
    * Returns null if the tag is not present/valid.
    */
    private TagObservation readMegaTag1(LLResult result) {
        if (result == null || !result.isValid()) {
            return null;
        }

        // Only use fiducials with id == TARGET_FIDUCIAL_ID
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) {
            return null;
        }

        LLResultTypes.FiducialResult tag = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == TARGET_FIDUCIAL_ID) {
                tag = f;
                break;
            }
        }
        if (tag == null) {
            return null; // MegaTag 1 not seen
        }

        // Raw angles from Limelight to the tag (degrees)
        double tx = tag.getTargetXDegrees();
        double ty = tag.getTargetYDegrees();

        // Robot yaw relative to tag: robot field yaw minus tag yaw
        double fieldYawDeg = 0.0;
        Pose3D botpose = result.getBotpose();
        if (botpose != null) {
            fieldYawDeg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        }
        double rzRelTagDeg = AngleUnit.normalizeDegrees(fieldYawDeg - TAG_YAW_DEG);

        return new TagObservation(
                tx,
                ty,
                rzRelTagDeg,
                TAG_X_IN,
                TAG_Y_IN,
                TAG_YAW_DEG
        );
    }

    /**
     * Converts Limelight angles + known tag pose into a field-space robot pose (inches, radians).
     */
    private Pose2d limelightToFieldPose(TagObservation obs) {
        double txRad = Math.toRadians(obs.txDeg);
        double distanceM = estimateDistanceMeters(obs.tyDeg);
        double distanceIn = DistanceUnit.METER.toInches(distanceM);

        // Robot offset from tag in tag frame:
        // forward (tag +X) and left (tag +Y) using camera ray
        double forwardIn = distanceIn * Math.cos(txRad);
        double leftIn = distanceIn * Math.sin(txRad);

        double tagYawRad = Math.toRadians(obs.tagYawDeg);

        // Rotate into field frame
        double fieldX = obs.tagXIn + forwardIn * Math.cos(tagYawRad) - leftIn * Math.sin(tagYawRad);
        double fieldY = obs.tagYIn + forwardIn * Math.sin(tagYawRad) + leftIn * Math.cos(tagYawRad);

        // Robot heading in field = tag yaw + RZ (robot yaw relative to tag)
        double headingRad = wrapToPi(Math.toRadians(obs.tagYawDeg + obs.rzDeg));

        return new Pose2d(fieldX, fieldY, headingRad);
    }

    /**
     * Estimate straight-line distance from camera to tag center using vertical angle.
     * Adjust CAMERA_HEIGHT_M, TAG_HEIGHT_M, CAMERA_PITCH_DEG for your robot.
     */
    private double estimateDistanceMeters(double tyDeg) {
        double tyRad = Math.toRadians(tyDeg);
        double pitchRad = Math.toRadians(CAMERA_PITCH_DEG);
        double denom = Math.tan(pitchRad + tyRad);
        if (Math.abs(denom) < 1e-6) {
            return Double.POSITIVE_INFINITY; // Avoid divide by zero
        }
        return (TAG_HEIGHT_M - CAMERA_HEIGHT_M) / denom;
    }

    private double wrapToPi(double angleRad) {
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }

    /**
     * Lightweight pose container (inches, radians) to expose current field pose.
     */
    public static class Pose2d {
        public double x;
        public double y;
        public double heading; // radians, normalized

        public Pose2d(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    /**
     * Container for a single MegaTag observation.
     */
    private static class TagObservation {
        final double txDeg;
        final double tyDeg;
        final double rzDeg;
        final double tagXIn;
        final double tagYIn;
        final double tagYawDeg;

        TagObservation(double txDeg, double tyDeg, double rzDeg,
                       double tagXIn, double tagYIn, double tagYawDeg) {
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.rzDeg = rzDeg;
            this.tagXIn = tagXIn;
            this.tagYIn = tagYIn;
            this.tagYawDeg = tagYawDeg;
        }
    }
}
