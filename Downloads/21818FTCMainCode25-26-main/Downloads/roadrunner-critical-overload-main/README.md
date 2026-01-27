package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


/**
* TeleOp OpMode for AprilTag detection and display.
* This OpMode continuously scans for AprilTags using a USB webcam and displays
* detection information on the Driver Station telemetry.
* Does not control any motors or servos - detection only.
  */
  @TeleOp(name="AprilTag Detector", group="Vision")
  public class AprilTagDetector extends LinearOpMode {

  // AprilTag processor for detecting tags
  AprilTagProcessor aprilTagProcessor;

  // Vision portal for camera access
  VisionPortal visionPortal;

  /**
    * Main run method - continuously scans for AprilTags and updates telemetry
      */
      @Override
      public void runOpMode() {

      // Initialize AprilTag detection system
      initAprilTagDetection();

      // Wait for the driver to press PLAY
      waitForStart();

      // Main loop - runs while OpMode is active
      while (opModeIsActive()) {

           // Get all currently detected AprilTags
           List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

           // Check if any tags were detected
           if (currentDetections.isEmpty()) {
               // No tags detected - display "NONE DETECTED"
               telemetry.addData("AprilTag", "NONE DETECTED");
               telemetry.update();
           } else {
               // Tags detected - process the first detected tag
               AprilTagDetection detection = currentDetections.get(0);
               int tagId = detection.id;
               
               // Display the tag ID
               telemetry.addData("AprilTag ID", tagId);
               
               // Determine obelisk position based on tag ID
               // Tag IDs 1, 2, 3 correspond to LEFT, CENTER, RIGHT positions
               String obeliskPosition = getObeliskPosition(tagId);
               telemetry.addData("Obelisk Position", obeliskPosition);
               
               // Update telemetry display
               telemetry.update();
           }
      }
      }

  /**
    * Initialize the AprilTag detection system.
    * Sets up the processor and vision portal with the USB webcam.
      */
      private void initAprilTagDetection() {
      // Create the AprilTag processor using the builder pattern
      aprilTagProcessor = new AprilTagProcessor.Builder().build();

      // Set image decimation to balance detection range and rate
      // Lower values = better range but slower processing
      aprilTagProcessor.setDecimation(2);

      // Create the vision portal to access the USB webcam
      // Assumes webcam is configured as "Webcam 1" in the robot configuration
      visionPortal = new VisionPortal.Builder()
      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
      .addProcessor(aprilTagProcessor)
      .build();
      }

  /**
    * Determines the obelisk position based on AprilTag ID.
    *
    * @param tagId The ID of the detected AprilTag
    * @return String indicating position: "LEFT", "CENTER", "RIGHT", or "UNKNOWN"
      */
      private String getObeliskPosition(int tagId) {
      switch (tagId) {
      case 1:
      return "LEFT";
      case 2:
      return "CENTER";
      case 3:
      return "RIGHT";
      default:
      return "UNKNOWN";
      }
      }
      }