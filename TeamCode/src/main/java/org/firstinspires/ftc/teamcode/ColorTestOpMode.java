package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "Color Percentage Detector")
public class ColorTestOpMode extends LinearOpMode {

    private OpenCvCamera webcam;
    private ColorDetectionPipeline pipeline;
    private volatile boolean cameraStarted = false;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();

        try {
            // Get webcam hardware
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

            // Create EasyOpenCV camera instance
            int cameraMonitorViewId = hardwareMap.appContext
                    .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            webcam = OpenCvCameraFactory.getInstance()
                    .createWebcam(webcamName, cameraMonitorViewId);

            // Create pipeline instance
            pipeline = new ColorDetectionPipeline();
            webcam.setPipeline(pipeline);

            // Open camera asynchronously
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    telemetry.addLine("Camera opened successfully!");
                    telemetry.update();

                    webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                    webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                    // Start FTC Dashboard camera stream
                    FtcDashboard.getInstance().startCameraStream(webcam, 30);
                    cameraStarted = true;
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Camera Error", "Error code: " + errorCode);
                    telemetry.addLine("Check camera connection and name");
                    telemetry.update();
                }
            });

            // Display instructions
            telemetry.addLine("Color Percentage Detection OpMode");
            telemetry.addLine("Automatically shows percentage of each color");
            telemetry.addLine("Waiting for camera...");
            telemetry.update();

            waitForStart();

            if (opModeIsActive()) {
                telemetry.clear();
                telemetry.addLine("OpMode Active - Analyzing colors...");
                telemetry.update();
            }

            while (opModeIsActive()) {
                // Display real-time color percentages
                telemetry.addData("Camera Status", cameraStarted ? "Running" : "Starting...");
                telemetry.addData("Frame Size", pipeline.getTotalPixels() + " pixels");
                telemetry.addLine("");

                // Color percentages with progress bars in telemetry
                telemetry.addData("Red Percentage", String.format("%.2f%%", pipeline.getRedPercentage()));
                telemetry.addData("Green Percentage", String.format("%.2f%%", pipeline.getGreenPercentage()));
                telemetry.addData("Blue Percentage", String.format("%.2f%%", pipeline.getBluePercentage()));
                telemetry.addLine("");

                telemetry.addData("Dominant Color", pipeline.getDominantColor());

                // Show color distribution as text bars
                addTelemetryBar("Red", pipeline.getRedPercentage());
                addTelemetryBar("Green", pipeline.getGreenPercentage());
                addTelemetryBar("Blue", pipeline.getBluePercentage());

                telemetry.addLine("");
                telemetry.addLine("Visual bars and percentages shown on camera feed");
                telemetry.update();

                // Small delay to prevent overwhelming the system
                sleep(100);
            }

        } catch (Exception e) {
            telemetry.addData("Error", "Exception occurred: " + e.getMessage());
            telemetry.update();
        } finally {
            // Cleanup
            if (webcam != null && cameraStarted) {
                webcam.stopStreaming();
                webcam.closeCameraDevice();
            }
        }
    }

    private void addTelemetryBar(String colorName, double percentage) {
        int barLength = 20;
        int filledBars = (int)Math.round(percentage / 5.0); // Each bar represents 5%
        if (filledBars > barLength) filledBars = barLength;

        StringBuilder bar = new StringBuilder();
        bar.append(colorName).append(" [");

        for (int i = 0; i < barLength; i++) {
            if (i < filledBars) {
                bar.append("█");
            } else {
                bar.append("░");
            }
        }
        bar.append("]");

        telemetry.addLine(bar.toString());
    }
}