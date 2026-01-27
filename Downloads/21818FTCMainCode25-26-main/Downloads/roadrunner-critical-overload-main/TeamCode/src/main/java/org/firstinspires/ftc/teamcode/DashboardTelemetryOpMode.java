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
 * Sends odometry + Limelight MegaTag 1 data to FTC Dashboard with a field overlay.
 * Self-contained: initializes dashboard, Pinpoint, and Limelight, and streams every loop.
 */
@TeleOp(name = "Dashboard Telemetry (LL + Odo)", group = "Sensor")
public class DashboardTelemetryOpMode extends LinearOpMode {

    // Only use MegaTag 1
    private static final int TARGET_FIDUCIAL_ID = 1;
    private static final int APRILTAG_PIPELINE = 2; // adjust to your LL pipeline index

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        // --- Hardware mapping ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // --- Pinpoint configuration (offsets from OdometryConstants) ---
        pinpoint.setOffsets(
                OdometryConstants.PINPOINT_OFFSET_DX_INCHES,
                OdometryConstants.PINPOINT_OFFSET_DY_INCHES,
                DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU(); // start clean

        // --- Limelight setup (start polling; set pipeline for MegaTag/AprilTag) ---
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        // --- FTC Dashboard init ---
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Dashboard telemetry ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry each loop
            pinpoint.update();

            // Read odometry
            double xIn = pinpoint.getPosX(DistanceUnit.INCH);
            double yIn = pinpoint.getPosY(DistanceUnit.INCH);
            double headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
            double headingRad = wrapToPi(Math.toRadians(headingDeg));

            // Read Limelight (only MegaTag 1)
            double txDeg = Double.NaN;
            double tyDeg = Double.NaN;
            double rzDeg = Double.NaN; // robot field yaw from botpose (if available)
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                LLResultTypes.FiducialResult tag = findTag(llResult, TARGET_FIDUCIAL_ID);
                if (tag != null) {
                    txDeg = tag.getTargetXDegrees();
                    tyDeg = tag.getTargetYDegrees();
                    if (llResult.getBotpose() != null) {
                        rzDeg = llResult.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
                    }
                }
            }

            // Driver Hub telemetry (quick sanity)
            telemetry.addData("X (in)", "%.2f", xIn);
            telemetry.addData("Y (in)", "%.2f", yIn);
            telemetry.addData("Heading (rad)", "%.2f", headingRad);
            telemetry.addData("tx (deg)", Double.isNaN(txDeg) ? "n/a" : String.format("%.2f", txDeg));
            telemetry.addData("ty (deg)", Double.isNaN(tyDeg) ? "n/a" : String.format("%.2f", tyDeg));
            telemetry.addData("RZ (deg)", Double.isNaN(rzDeg) ? "n/a" : String.format("%.2f", rzDeg));
            telemetry.update();

            // Dashboard packet (guaranteed send every loop)
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x_in", xIn);
            packet.put("y_in", yIn);
            packet.put("heading_rad", headingRad);
            packet.put("tx_deg", txDeg);
            packet.put("ty_deg", tyDeg);
            packet.put("rz_deg", rzDeg);

            // Field overlay: draw robot as a rectangle with heading arrow
            Canvas field = packet.fieldOverlay();
            drawRobot(field, xIn, yIn, headingRad, 9.0, 9.0);

            dashboard.sendTelemetryPacket(packet);
        }

        limelight.stop();
    }

    /**
     * Find the first fiducial with the given id; ignore others.
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
     * Draw a rectangle robot and heading arrow on the dashboard field overlay.
     */
    private void drawRobot(Canvas canvas, double x, double y, double headingRad, double halfLength, double halfWidth) {
        double[][] corners = {
                { halfLength,  halfWidth},
                { halfLength, -halfWidth},
                {-halfLength, -halfWidth},
                {-halfLength,  halfWidth}
        };
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        double[][] world = new double[4][2];
        for (int i = 0; i < 4; i++) {
            double rx = corners[i][0] * cos - corners[i][1] * sin;
            double ry = corners[i][0] * sin + corners[i][1] * cos;
            world[i][0] = x + rx;
            world[i][1] = y + ry;
        }

        for (int i = 0; i < 4; i++) {
            double x1 = world[i][0];
            double y1 = world[i][1];
            double x2 = world[(i + 1) % 4][0];
            double y2 = world[(i + 1) % 4][1];
            canvas.strokeLine(x1, y1, x2, y2);
        }

        // Heading arrow
        double arrowLen = halfLength * 1.5;
        double hx = x + arrowLen * cos;
        double hy = y + arrowLen * sin;
        canvas.setStrokeWidth(2);
        canvas.strokeLine(x, y, hx, hy);
    }

    /**
     * Normalize angle to [-π, π].
     */
    private double wrapToPi(double angleRad) {
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }
}
