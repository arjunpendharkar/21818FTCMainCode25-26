package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp for controlling spindexer servo
 * Hold X: continuous rotation
 * Tap Y: move to 88 degrees (shortest path)
 * No buttons: don't move at all
 */
@TeleOp(name = "Spindexer Code Teleop", group = "TeleOp")
public class spindexerCodeteleop extends LinearOpMode {
    // Declare hardware
    private Servo spindexer;
    private AnalogInput angleEncoder;

    // Continuous rotation servo positions
    private static final double SPIN_CW = 0.3;   // Spin clockwise direction
    private static final double SPIN_CCW = 0.7;  // Spin counter-clockwise direction
    private static final double STOP_POSITION = 0.5;  // Stop position

    // Target angle
    private static final double TARGET_ANGLE = 88.0;
    private static final double ANGLE_TOLERANCE = 3.0;  // Stop when within 3 degrees
    private static final double POSITION_SPEED = 0.35;  // Speed for positioning
    
    // Modes
    private enum Mode {
        IDLE,              // No buttons pressed - don't move
        CONTINUOUS_ROTATION,  // Holding X - continuous rotation
        POSITIONING        // Tapped Y - move to target
    }
    private Mode currentMode = Mode.IDLE;

    // Button state tracking
    private boolean xButtonCurrent = false;
    private boolean yButtonPrevious = false;
    private boolean yButtonCurrent = false;

    // Analog encoder calibration constants
    private static final double MAX_VOLTAGE = 5.0;
    private static final double MAX_DEGREES = 360.0;

    @Override
    public void runOpMode() {
        // Initialize spindexer servo
        try {
            spindexer = hardwareMap.get(Servo.class, "spindexerservo");
        } catch (Exception e) {
            telemetry.addData("Error", "Servo not found. Make sure it's configured.");
        }

        // Initialize analog input encoder
        angleEncoder = hardwareMap.get(AnalogInput.class, "axonspindexer");

        // Initialize servo to stop position
        if (spindexer != null) {
            spindexer.setPosition(STOP_POSITION);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Hold X: rotate | Tap Y: go to 88° | No buttons: stop");
        telemetry.update();

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Update button states
            xButtonCurrent = gamepad1.x;
            yButtonCurrent = gamepad1.y;
            
            // Detect Y button tap
            boolean yButtonTapped = false;
            if (yButtonCurrent && !yButtonPrevious) {
                yButtonTapped = true;
            }
            yButtonPrevious = yButtonCurrent;

            // Mode switching logic
            if (xButtonCurrent) {
                // X is pressed - continuous rotation
                currentMode = Mode.CONTINUOUS_ROTATION;
            } else if (yButtonTapped) {
                // Y was tapped - switch to positioning mode
                currentMode = Mode.POSITIONING;
            }
            // If neither button is pressed, stay in current mode (will stop when target reached)

            // Read current angle from encoder
            double voltage = angleEncoder.getVoltage();
            double currentAngle = (voltage / MAX_VOLTAGE) * MAX_DEGREES;
            currentAngle = currentAngle % 360.0;
            if (currentAngle < 0) {
                currentAngle += 360.0;
            }

            // Control servo based on mode
            if (spindexer != null) {
                if (currentMode == Mode.CONTINUOUS_ROTATION) {
                    // Holding X - continuous rotation
                    spindexer.setPosition(SPIN_CW);
                } else if (currentMode == Mode.POSITIONING) {
                    // Positioning mode: move toward 88 degrees using shortest path
                    double angleError = calculateShortestAngle(currentAngle, TARGET_ANGLE);
                    
                    // Check if within tolerance
                    if (Math.abs(angleError) <= ANGLE_TOLERANCE) {
                        // At target - stop
                        spindexer.setPosition(STOP_POSITION);
                        currentMode = Mode.IDLE;
                    } else {
                        // Move toward target using shortest path
                        if (angleError > 0) {
                            // Need to rotate counter-clockwise (increase angle)
                            // Positive error means target is CCW from current
                            spindexer.setPosition(STOP_POSITION + POSITION_SPEED);
                        } else {
                            // Need to rotate clockwise (decrease angle)
                            // Negative error means target is CW from current
                            spindexer.setPosition(STOP_POSITION - POSITION_SPEED);
                        }
                    }
                } else {
                    // IDLE mode - STOP completely
                    spindexer.setPosition(STOP_POSITION);
                }
            }

            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Mode", currentMode == Mode.IDLE ? "IDLE (Stopped)" : 
                              currentMode == Mode.CONTINUOUS_ROTATION ? "Continuous Rotation" : "Positioning to 88°");
            telemetry.addData("X Button", xButtonCurrent ? "Held (Rotating)" : "Not Pressed");
            telemetry.addData("Y Button", yButtonCurrent ? "Pressed" : "Not Pressed");
            
            if (currentMode == Mode.POSITIONING) {
                double angleError = calculateShortestAngle(currentAngle, TARGET_ANGLE);
                telemetry.addData("Target Angle", "88°");
                telemetry.addData("Angle Error", "%.1f°", angleError);
                telemetry.addData("At Target", Math.abs(angleError) <= ANGLE_TOLERANCE ? "YES" : "NO");
            }
            
            telemetry.addData("Current Angle", "%.2f°", currentAngle);
            telemetry.addData("Encoder Voltage", "%.4f V", voltage);
            telemetry.update();
        }
    }

    /**
     * Calculate the shortest angular distance from current to target
     * Returns positive if target is counter-clockwise from current (need to increase angle)
     * Returns negative if target is clockwise from current (need to decrease angle)
     */
    private double calculateShortestAngle(double current, double target) {
        double error = target - current;
        
        // Normalize to -180 to 180 range (shortest path)
        while (error > 180.0) {
            error -= 360.0;
        }
        while (error < -180.0) {
            error += 360.0;
        }
        
        return error;
    }
}
