package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

/**
 * Simple viewer OpMode that streams odometry + Limelight data to FTC Dashboard.
 * - Displays X/Y/Heading (from Pinpoint) and tx/ty/RZ (from Limelight fiducial).
 * - Includes an optional 2D overlay of the robot pose on a field.
 */
@TeleOp(name = "Dashboard Odometry Viewer", group = "Sensor")
public class DashboardOdometryViewer extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    // Use MegaTag 1 by default; change if needed.
    private static final int TARGET_FIDUCIAL_ID = 1;

    @Override
    public void runOpMode() {
        // Map hardware
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure Pinpoint offsets from OdometryConstants and reset pose/IMU
        pinpoint.setOffsets(
                OdometryConstants.PINPOINT_OFFSET_DX_INCHES,
                OdometryConstants.PINPOINT_OFFSET_DY_INCHES,
                DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // Start Limelight polling (assumes pipeline already set in LL UI or prior code)
        limelight.start();

        // Init FTC Dashboard
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Dashboard viewer ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry
            pinpoint.update();

            double xIn = pinpoint.getPosX(DistanceUnit.INCH);
            double yIn = pinpoint.getPosY(DistanceUnit.INCH);
            double headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

            // Grab Limelight data (tx/ty/RZ) for target fiducial
            LLResult llResult = limelight.getLatestResult();
            double txDeg = Double.NaN;
            double tyDeg = Double.NaN;
            double rzDeg = Double.NaN;
            if (llResult != null && llResult.isValid()) {
                LLResultTypes.FiducialResult tag = findTag(llResult, TARGET_FIDUCIAL_ID);
                if (tag != null) {
                    txDeg = tag.getTargetXDegrees();
                    tyDeg = tag.getTargetYDegrees();
                    // RZ: robot yaw relative to field (botpose yaw) relative to tag yaw if provided
                    double fieldYaw = llResult.getBotpose() != null
                            ? llResult.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES)
                            : Double.NaN;
                    rzDeg = fieldYaw;
                }
            }

            // Send to Driver Hub telemetry (optional)
            telemetry.addData("X (in)", "%.2f", xIn);
            telemetry.addData("Y (in)", "%.2f", yIn);
            telemetry.addData("Heading (deg)", "%.1f", headingDeg);
            telemetry.addData("tx (deg)", Double.isNaN(txDeg) ? "n/a" : String.format("%.2f", txDeg));
            telemetry.addData("ty (deg)", Double.isNaN(tyDeg) ? "n/a" : String.format("%.2f", tyDeg));
            telemetry.addData("RZ (deg)", Double.isNaN(rzDeg) ? "n/a" : String.format("%.2f", rzDeg));
            telemetry.update();

            // Build dashboard packet
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x_in", xIn);
            packet.put("y_in", yIn);
            packet.put("heading_deg", headingDeg);
            packet.put("tx_deg", txDeg);
            packet.put("ty_deg", tyDeg);
            packet.put("rz_deg", rzDeg);

            // Optional 2D overlay: simple arrow for robot on a 144x144 inch field
            Canvas field = packet.fieldOverlay();
            drawRobot(field, xIn, yIn, Math.toRadians(headingDeg), 9.0 /* robot half-length */, 9.0 /* half-width */);

            dashboard.sendTelemetryPacket(packet);
        }

        limelight.stop();
    }

    /**
     * Finds the first fiducial with the given id in the Limelight result.
     */
    private LLResultTypes.FiducialResult findTag(LLResult result, int targetId) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == targetId) {
                return f;
            }
        }
        return null;
    }

    /**
     * Draw a simple rectangle + heading line for the robot.
     * Field units are inches in Dashboard's default coordinate space.
     */
    private void drawRobot(Canvas canvas, double x, double y, double headingRad, double halfLength, double halfWidth) {
        // Robot corners relative to center
        double[][] corners = {
                { halfLength,  halfWidth},
                { halfLength, -halfWidth},
                {-halfLength, -halfWidth},
                {-halfLength,  halfWidth}
        };
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        // Rotate + translate corners
        double[][] world = new double[4][2];
        for (int i = 0; i < 4; i++) {
            double rx = corners[i][0] * cos - corners[i][1] * sin;
            double ry = corners[i][0] * sin + corners[i][1] * cos;
            world[i][0] = x + rx;
            world[i][1] = y + ry;
        }

        // Draw outline
        for (int i = 0; i < 4; i++) {
            double x1 = world[i][0];
            double y1 = world[i][1];
            double x2 = world[(i + 1) % 4][0];
            double y2 = world[(i + 1) % 4][1];
            canvas.strokeLine(x1, y1, x2, y2);
        }

        // Draw heading arrow from center forward
        double arrowLen = halfLength * 1.5;
        double hx = x + arrowLen * cos;
        double hy = y + arrowLen * sin;
        canvas.setStrokeWidth(2);
        canvas.strokeLine(x, y, hx, hy);
    }
}
