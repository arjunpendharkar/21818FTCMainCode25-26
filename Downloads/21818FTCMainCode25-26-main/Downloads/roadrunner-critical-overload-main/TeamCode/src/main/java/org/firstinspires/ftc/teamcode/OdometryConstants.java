package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class OdometryConstants {

    // --- Motor and Encoder Specifications ---
    // GoBILDA Yellow Jacket 19.2:1 motors are common. Their internal encoder is 537.6 counts per revolution.
    public static final double TICKS_PER_MOTOR_REV = 537.6; 
    public static final double GEAR_REDUCTION_EXTERNAL = 24.0 / 16.0; // 24-tooth motor gear to 16-tooth wheel gear
    public static final double TOTAL_GEAR_REDUCTION = 19.2 * GEAR_REDUCTION_EXTERNAL; // Internal * External
    
    // Wheel Specifications
    public static final double WHEEL_DIAMETER_MM = 104.0;
    public static final double WHEEL_RADIUS_INCHES = (WHEEL_DIAMETER_MM / 2.0) / 25.4; // Convert mm to inches

    // --- Calibrated Odometry Values ---
    // This is your empirically measured value and is generally the most accurate for linear movement.
    public static final double CALIBRATED_TICKS_PER_INCH = 1380.0; 

    // --- Robot Geometry (in Inches) ---
    // Distance between the centers of the left and right wheels (track width).
    // Used for calculating robot rotation.
    public static final double TRACK_WIDTH_INCHES = 273.950 / 25.4; // 273.950 mm converted to inches
    
    // Distance between the centers of the front and back wheels on one side.
    // Important for Mecanum strafing odometry.
    public static final double LATERAL_DISTANCE_INCHES = 380.0 / 25.4; // 380 mm converted to inches

    // --- Odometry Wheel Configuration ---
    // Assuming your drive motors are also your odometry encoders.
    // If you were using dedicated non-powered odometry pods, these would be different.
    // For a standard Mecanum setup, all four drive motor encoders can be used.
    public static final String FRONT_LEFT_MOTOR_NAME = "frontLeft";
    public static final String FRONT_RIGHT_MOTOR_NAME = "frontRight";
    public static final String BACK_LEFT_MOTOR_NAME = "backLeft";
    public static final String BACK_RIGHT_MOTOR_NAME = "backRight";

    // Odometry update rate (how often to poll encoders)
    public static final long ODOMETRY_UPDATE_PERIOD_MS = 20; // Update every 20 milliseconds

    // Encoder direction multipliers - adjust if an encoder counts negatively when robot moves forward.
    // For a mecanum drive, you might need to adjust these based on motor directions and encoder wiring.
    public static final double LEFT_ENCODER_MULTIPLIER = 1.0;
    public static final double RIGHT_ENCODER_MULTIPLIER = 1.0;
    public static final double STRAFE_ENCODER_MULTIPLIER = 1.0; // For lateral movement (e.g., backLeft or backRight for strafing)

    // --- GoBilda Pinpoint (2-wheel odometry) configuration ---
    // These offsets are the pod positions *relative to the tracking point* (usually robot center).
    //
    // Pinpoint convention (matches FTC sample):
    // - dX: sideways offset of the X/forward pod. Left of center is +, right of center is -.
    // - dY: forward/back offset of the Y/strafe pod. Forward of center is +, backward is -.
    //
    // TODO: Measure on your robot and update these. Using 0,0 will usually cause heading/position error.
    public static final double PINPOINT_OFFSET_DX_INCHES = 6.0;
    public static final double PINPOINT_OFFSET_DY_INCHES = 6.7;

    // Example calculation for theoretical ticks per inch (for reference, if you want to compare)
    // This is often less accurate than CALIBRATED_TICKS_PER_INCH due to real-world factors.
    public static final double THEORETICAL_TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * TOTAL_GEAR_REDUCTION;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = 2 * Math.PI * WHEEL_RADIUS_INCHES;
    public static final double THEORETICAL_TICKS_PER_INCH = THEORETICAL_TICKS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_INCHES;

}
