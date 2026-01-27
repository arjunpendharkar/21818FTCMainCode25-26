package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "Drive + Flywheel + Intake + Axon + Outtake + Spindexer")
public class FullRobotTeleop extends LinearOpMode {

    // ===== Drive motors =====
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== Flywheel servos (continuous rotation) =====
    CRServo flywheelServoLeft, flywheelServoRight;

// ===== Intake motor =====
    DcMotor intakeMotor;
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;

    // Limelight AprilTag pipeline + staleness guard (for odometry resync)
    private static final int APRILTAG_PIPELINE = 2;
    private static final long MAX_TAG_STALENESS_MS = 200;
    private boolean lastBack = false;
    private Pose2D lastLimelightPose = null;
    private boolean lastResyncSuccess = false;
    private ElapsedTime lastResyncTimer = new ElapsedTime();
    private boolean seededFromLimelight = false;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // ===== Flywheel toggle =====
    boolean flywheelOn = false;
    boolean lastAState = false;

    // ===== Intake toggle =====
    boolean intakeOn = false;
    boolean lastBState = false;
    boolean intakeReversed = false; // Direction: false = forward (1.0), true = reverse (-1.0)
    boolean lastRightBumperState = false;
    // ===== Spindexer servo =====
    CRServo spindexer;
    AnalogInput axonAnalog;

    // Continuous rotation servo power values (CRServo uses -1.0 to 1.0, 0.0 = stop)
    // Negative power = LEFT (decreasing angle), Positive power = RIGHT (increasing angle)
    private static final double SPIN_POWER = -0.15;   // Spin power for continuous rotation LEFT (X button) - super slow
    private static final double STOP_POWER = 0.0;    // Stop power

    // Spindexer Y button timing control
    private static final double INIT_ANGLE = 98.0;  // Initial position target (98°, acceptable range 97-99°)
    private static final double INIT_ANGLE_MIN = 97.0;  // Minimum acceptable initial angle
    private static final double INIT_ANGLE_MAX = 99.0;  // Maximum acceptable initial angle
    private static final double FIRST_Y_RUN_TIME = 0.2;  // First Y press: run for 0.2 seconds
    private static final double Y_BUTTON_RUN_TIME = 0.52;  // Subsequent Y presses: run for 0.65 seconds
    private static final double MAX_DEGREES_SPINDEXER = 230.0;  // Max rotation (230° hardware limit)
    private static final double MAX_VOLTAGE = 5.0;  // Analog encoder max voltage
    private static final double ANGLE_TOLERANCE = 3.0;  // Stop when within 3 degree
    // Spindexer state tracking
    private enum SpindexerInitState {
        MOVING_TO_INIT,    // Moving to 97-99° position
        INITIALIZED        // Initialization complete
    }
    // Start initialized so spindexer/outtake controls respond immediately
    private SpindexerInitState initState = SpindexerInitState.INITIALIZED;
    private ElapsedTime yButtonTimer = new ElapsedTime();
    private boolean yButtonRunning = false;
    private boolean spindexerYButtonPrevious = false;
    private boolean isFirstYPress = true;  // Track if this is the first Y press


    // ===== Outtake servo (rack and pinion control) =====
    // Axon MK2 Max servo configured in POSITIONAL mode
    // Connected to goBILDA Servo-Driven Gear Rack Kit (SKU: 3207-0002-0002)
    // Full travel: 240mm (9.45 inches) - PWM range: 700-2300μsec, 0.15mm/μsec
    // Positional mode uses setPosition() to move to specific positions (0.0 to 1.0)
    // 0.0 = fully retracted (motors closest), 1.0 = fully extended (motors farthest)
    Servo outtakeServo;

    // Ball diameter constant (for spacing calculation)
    private static final double BALL_DIAMETER_INCHES = 4.9;  // Ball diameter in inches

    // Rack and pinion specifications (from goBILDA kit)
    private static final double FULL_TRAVEL_INCHES = 9.45;  // Full travel distance: 240mm = 9.45 inches

    // Position values for linear actuator control (Servo uses 0.0 to 1.0 range)
    // STARTING_POSITION: Minimum position - servo cannot go more inward than this
    // EXTENDED_POSITION: Maximum extended position (outside) - absolute maximum
    // Right D-pad: extends to EXTENDED_POSITION (fully extended outward)
    // Left D-pad: returns to STARTING_POSITION (minimum - cannot go further in)
    private static final double EXTENDED_POSITION = 1.0;   // Maximum extended position (outside) - absolute maximum
    private static final double STARTING_POSITION = 0.595;  // Minimum position - slightly tighter than physical position for better ball grip (cannot go further in)

    // Button state tracking for edge detection
    private boolean lastRightDpadState = false;

    // ===== Shooting Mode (Right Trigger = shoot, Left Trigger/Left D-pad = exit) =====
    // Shooting sequence: 1) Close outtake servo (IN)  2) Spin up flywheels  3) Spin spindexer to shoot
    // IDLE state: Outtake is OUT/extended, flywheels OFF, ready to intake
    // SHOOTING states: Outtake is IN/closed, flywheels ON, spindexer advances per trigger
    private enum ShootingState {
        IDLE,                    // Not in shooting mode - outtake OUT, flywheels OFF
        CLOSING_OUTTAKE,         // Step 1: Outtake servo closing IN (coming together)
        SPINNING_UP_FLYWHEELS,   // Step 2: Flywheels spinning up
        READY_TO_SHOOT,          // Ready to fire - waiting for trigger tap
        SHOOTING                 // Step 3: Spindexer spinning to shoot one ball
    }
    private ShootingState shootingState = ShootingState.IDLE;
    private boolean lastRightTriggerState = false;  // For edge detection on right trigger
    private ElapsedTime shootSequenceTimer = new ElapsedTime();  // Timer for sequence steps
    private ElapsedTime shootSpindexerTimer = new ElapsedTime();
    private boolean isFirstShot = true;  // Track if this is the first shot in shooting mode
    private boolean rtSpinActive = false; // Track timed spin on right trigger
    private static final double SHOOT_SPINDEXER_TIME = 0.52;  // Time to spin spindexer for each shot (one index)
    private static final double OUTTAKE_CLOSE_TIME = 0.5;     // Time for outtake servo to close IN (seconds)
    private static final double FLYWHEEL_SPINUP_TIME = 0.75;  // Time for flywheels to spin up (seconds)

    // ===== Test Servo (5-turn servo) =====
    // Expansion hub port 5, name "testServo", type Servo
    // Left trigger: hold to move fully (1.0), release to retract (0.0)
    Servo testServo;

    private static final int GREEN_PIPELINE_INDEX = 0;        // Fixed pipeline for green balls
    private static final double MIN_TARGET_AREA = 1.0;        // Filter tiny/noisy blobs (% of image)
    private static final long MAX_LL_STALENESS_MS = 150;      // Reject old frames for green pipeline
    private static final double BOTTOM_MARGIN = 2.0;          // ty within this of recent max counts as bottom
    private static final double TY_MAX_DECAY_PER_SEC = 5.0;   // How fast recent max ty decays when no target
    private static final double BALL_COOLDOWN_SEC = 0.8;      // Debounce interval after detection
    private static final double AUTO_INTAKE_RUN_TIME = 3.0;   // Seconds to run intake after detection
    private static final double AUTO_INTAKE_POWER = 1.0;      // Intake power during auto run
    private static final double CENTER_TX_MARGIN = 2.0;       // Require tx near center before counting disappearance
    private static final double TY_DROP_THRESHOLD = -0.5;     // Require ty to be negative (dropping) before loss
    private int desiredPipeline = GREEN_PIPELINE_INDEX;
    private double recentTyMax = -999.0; // Track bottom-most ty observed recently
    private boolean intakeAutoRunning = false;
    private ElapsedTime intakeRunTimer = new ElapsedTime();
    private ElapsedTime detectionCooldownTimer = new ElapsedTime();
    private ElapsedTime visionLoopTimer = new ElapsedTime();
    private double lastLimelightTy = 0.0;
    private double lastLimelightTx = 0.0;
    private boolean lastTargetVisible = false;

    @Override
    public void runOpMode() {

// ===== Drive hardware =====
        frontLeft= hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft= hardwareMap.get(DcMotor.class, "backLeft");
        backRight= hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

// ===== Flywheel hardware (CRServos) =====
        flywheelServoLeft = hardwareMap.get(CRServo.class, "flywheelServoLeft");
        flywheelServoRight = hardwareMap.get(CRServo.class, "flywheelServoRight");
        flywheelServoLeft.setDirection(CRServo.Direction.REVERSE);

// ===== Intake hardware =====
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

// ===== Pinpoint odometry hardware =====
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

// ===== Spindexer hardware =====
        spindexer = hardwareMap.get(CRServo.class, "axonservo1");
        spindexer.setPower(STOP_POWER);
        axonAnalog = hardwareMap.get(AnalogInput.class, "axonspindexer");
        // Start initialized so spindexer/outtake controls respond immediately
        initState = SpindexerInitState.INITIALIZED;
        yButtonRunning = false;
        isFirstYPress = true;

// ===== Outtake servo hardware (GoBilda 5-turn speed servo in Positional mode) =====
        // MUST use Servo (not CRServo) because the servo is configured in positional mode
        // setPosition() is used to move to specific positions (0.0 to 1.0)
        outtakeServo = hardwareMap.get(Servo.class, "outtakeServo");
        // Reverse servo direction if needed (uncomment if servo moves in wrong direction)
        outtakeServo.setDirection(Servo.Direction.REVERSE);
        // INIT: Outtake servo starts fully OUT/extended - guarantees known starting state
        // This is the same position as right d-pad
        outtakeServo.setPosition(EXTENDED_POSITION);

// ===== Test Servo hardware (5-turn servo) =====
        testServo = hardwareMap.get(Servo.class, "testServo");
        testServo.setPosition(0.0); // Start at fully retracted position

// ===== Limelight hardware =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(GREEN_PIPELINE_INDEX); // lock to green pipeline
        limelight.start(); // begin polling
        lastResyncTimer.reset();

        waitForStart();
        visionLoopTimer.reset();
        detectionCooldownTimer.reset();
        intakeRunTimer.reset();

        // Try a one-time initial seed from AprilTag pose (does not loop)
        attemptLimelightInitialSeed();

        while (opModeIsActive()) {

            // Keep Pinpoint updated even if we only resync manually
            pinpoint.update();

            // Manual resync from Limelight AprilTag pose (gamepad1.back)
            boolean backPressed = gamepad1.back;
            boolean backTapped = backPressed && !lastBack;
            if (backTapped) {
                attemptLimelightResync();
            }
            lastBack = backPressed;

// ===== Drive (rotation unchanged) =====
            double forward = -gamepad1.left_stick_y;
            double strafe= gamepad1.left_stick_x;
            double turn= gamepad1.right_stick_x;

            frontLeft.setPower(forward + strafe + turn);
            frontRight.setPower(forward - strafe - turn);
            backLeft.setPower(forward - strafe + turn);
            backRight.setPower(forward + strafe - turn);

// ===== Flywheel toggle (A) or Shooting Mode =====
            boolean aPressed = gamepad1.a;
            if (aPressed && !lastAState) {
                flywheelOn = !flywheelOn;
            }
            lastAState = aPressed;

            

            // Flywheel runs if toggled on OR if in shooting mode
            // In shooting mode: flywheels run during CLOSING_OUTTAKE (to pre-spool), SPINNING_UP, READY, SHOOTING
            // Flywheels turn OFF when returning to IDLE
            boolean shootingFlywheelsOn = (shootingState == ShootingState.CLOSING_OUTTAKE ||
                                           shootingState == ShootingState.SPINNING_UP_FLYWHEELS || 
                                           shootingState == ShootingState.READY_TO_SHOOT || 
                                           shootingState == ShootingState.SHOOTING);
            boolean flywheelShouldRun = flywheelOn || shootingFlywheelsOn;
            flywheelServoLeft.setPower(flywheelShouldRun ? 1.0 : 0);
            flywheelServoRight.setPower(flywheelShouldRun ? 1.0 : 0);

// ===== Intake toggle (B = on/off, Right Bumper = reverse direction) =====
            // Toggle intake on/off with B button
            boolean bPressed = gamepad1.b;
            if (bPressed && !lastBState) {
                intakeOn = !intakeOn;
            }
            lastBState = bPressed;

            // Toggle intake direction with right bumper
            boolean rightBumperPressed = gamepad1.right_bumper;
            if (rightBumperPressed && !lastRightBumperState) {
                intakeReversed = !intakeReversed; // Toggle direction
            }
            lastRightBumperState = rightBumperPressed;

            // Set intake power: on/off controlled by B, direction controlled by right bumper
            double manualIntakePower = intakeOn ? (intakeReversed ? -1.0 : 1.0) : 0.0;
            double commandedIntakePower = manualIntakePower;  // May be overridden by auto intake

// ===== Spindexer control (X = continuous left rotation, Y = timed rotation, Right Trigger = shoot) =====
            // Read current angle from encoder (0° to 230° range - hardware limitation)
            double currentVoltage = axonAnalog.getVoltage();
            double currentAngle = (currentVoltage / MAX_VOLTAGE) * MAX_DEGREES_SPINDEXER;
            // Clamp to valid physical range (0° to 230°)
            if (currentAngle < 0) {
                currentAngle = 0;
            } else if (currentAngle > MAX_DEGREES_SPINDEXER) {
                currentAngle = MAX_DEGREES_SPINDEXER;
            }

            // X button: Hold for continuous LEFT rotation, release to stop
            boolean xButtonPressed = gamepad1.x;
            
            // Y button: Tap to run for 0.5 seconds
            boolean spindexerYButtonCurrent = gamepad1.y;
            boolean yButtonTapped = false;
            if (spindexerYButtonCurrent && !spindexerYButtonPrevious) {
                yButtonTapped = true;
            }
            spindexerYButtonPrevious = spindexerYButtonCurrent;
            
            // Right Trigger: hold to close outtake and spin flywheels
            double rightTrigger = gamepad1.right_trigger;
            boolean rightTriggerPressed = rightTrigger > 0.5;  // Treat >50% as pressed
            
            // Left Trigger OR Left D-pad: Exit shooting mode
            double leftTrigger = gamepad1.left_trigger;
            boolean leftDpadForExit = gamepad1.dpad_left;
            boolean exitShootingMode = (leftTrigger > 0.5) || leftDpadForExit;
            
            // Trigger-based behavior:
            // - Right trigger tap: close outtake, spin flywheels, and spin spindexer for 0.55s
            // - Left trigger/dpad-left: open outtake, stop flywheels and spindexer
            if (exitShootingMode) {
                shootingState = ShootingState.IDLE;
                isFirstShot = true;
                flywheelOn = false;
                yButtonRunning = false;
                rtSpinActive = false;
                spindexer.setPower(STOP_POWER);
                outtakeServo.setPosition(EXTENDED_POSITION);
            } else if (rightTriggerPressed && !lastRightTriggerState) {
                shootingState = ShootingState.SHOOTING;
                flywheelOn = true;
                outtakeServo.setPosition(STARTING_POSITION);
                rtSpinActive = true;
                shootSpindexerTimer.reset();
            }
            
            // Check if Y button was tapped to start timed run (only if not in shooting mode)
            if (yButtonTapped && !yButtonRunning && shootingState == ShootingState.IDLE) {
                yButtonRunning = true;
                yButtonTimer.reset();
            }
            
            // Handle shooting mode spindexer spin (highest priority)
            if (shootingState == ShootingState.SHOOTING) {
                // Run spindexer for a time d burst on RT tap
                double shootRunTime = 0.55; // seconds
                if (rtSpinActive) {
                    spindexer.setPower(SPIN_POWER);
                    if (shootSpindexerTimer.seconds() >= shootRunTime) {
                        rtSpinActive = false;
                        spindexer.setPower(STOP_POWER);
                        shootingState = ShootingState.READY_TO_SHOOT;
                    }
                } else {
                    spindexer.setPower(STOP_POWER);
                }
            } else if (shootingState == ShootingState.CLOSING_OUTTAKE) {
                spindexer.setPower(STOP_POWER);
            } else if (shootingState == ShootingState.SPINNING_UP_FLYWHEELS) {
                spindexer.setPower(STOP_POWER);
            } else if (shootingState == ShootingState.READY_TO_SHOOT) {
                spindexer.setPower(STOP_POWER);
            } else if (yButtonRunning) {
                double runTime = isFirstYPress ? FIRST_Y_RUN_TIME : Y_BUTTON_RUN_TIME;
                if (yButtonTimer.seconds() >= runTime) {
                    spindexer.setPower(STOP_POWER);
                    yButtonRunning = false;
                    isFirstYPress = false;
                } else {
                    spindexer.setPower(SPIN_POWER);
                }
            } else if (xButtonPressed) {
                spindexer.setPower(SPIN_POWER);
            } else {
                spindexer.setPower(STOP_POWER);
            }

// ===== Limelight vision: green only, auto-intake on bottom disappearance =====
            // Force green pipeline
            if (getLimelightPipeline() != GREEN_PIPELINE_INDEX) {
                setLimelightPipeline(GREEN_PIPELINE_INDEX);
            }

            LLResult llResult = limelight.getLatestResult();
            boolean targetVisible = false;
            double llTx = 0.0;
            double llTy = 0.0;
            double llTa = 0.0;
            long llAgeMs = -1;
            if (llResult != null) {
                llAgeMs = llResult.getStaleness();
                llTa = llResult.getTa();
                boolean fresh = llAgeMs >= 0 && llAgeMs <= MAX_LL_STALENESS_MS;
                boolean validBlob = llResult.isValid() && llTa >= MIN_TARGET_AREA;
                targetVisible = fresh && validBlob;
                if (targetVisible) {
                    llTx = llResult.getTx();
                    llTy = llResult.getTy();
                }
            }

            // Update vision timing for decay of bottom tracker
            double dtVision = visionLoopTimer.seconds();
            visionLoopTimer.reset();
            double decay = TY_MAX_DECAY_PER_SEC * dtVision;
            if (targetVisible) {
                if (recentTyMax < -900.0) {
                    recentTyMax = llTy;
                } else {
                    recentTyMax = Math.max(llTy, recentTyMax - decay);
                }
            } else {
                recentTyMax = recentTyMax - decay;
            }

            // Detect disappearance near bottom AND centered to trigger auto intake
            boolean nearBottomLastFrame = lastLimelightTy >= (recentTyMax - BOTTOM_MARGIN);
            boolean centeredLastFrame = Math.abs(lastLimelightTx) <= CENTER_TX_MARGIN;
            boolean wasDropping = lastLimelightTy <= TY_DROP_THRESHOLD; // negative ty means moving down/below center
            boolean tvFell = lastTargetVisible && !targetVisible;
            boolean cooldownActive = detectionCooldownTimer.seconds() < BALL_COOLDOWN_SEC;
            if (tvFell && nearBottomLastFrame && centeredLastFrame && wasDropping && !cooldownActive) {
                intakeAutoRunning = true;
                intakeRunTimer.reset();
                detectionCooldownTimer.reset();
                commandedIntakePower = AUTO_INTAKE_POWER;
            }

            if (intakeAutoRunning) {
                commandedIntakePower = AUTO_INTAKE_POWER;
                if (intakeRunTimer.seconds() >= AUTO_INTAKE_RUN_TIME) {
                    intakeAutoRunning = false;
                    commandedIntakePower = manualIntakePower; // return control to driver
                }
            }

            // Apply intake command (manual or auto)
            intakeMotor.setPower(commandedIntakePower);
            if (targetVisible) {
                lastLimelightTy = llTy;
                lastLimelightTx = llTx;
            }
            lastTargetVisible = targetVisible;

// ===== Outtake servo D-pad control (GoBilda 5-turn speed servo in Positional mode) =====
            // Right D-pad: Manual extend OUT (same as init position)
            // Left D-pad: Exit shooting mode (handled above with left trigger)
            boolean rightDpadPressed = gamepad1.dpad_right;
            
            // Detect D-pad right tap (edge detection - only trigger on press, not while held)
            boolean rightDpadTapped = false;
            if (rightDpadPressed && !lastRightDpadState) {
                rightDpadTapped = true;
            }
            lastRightDpadState = rightDpadPressed;
            
            // Right D-pad: Manual extend OUT and exit shooting mode if active
            if (rightDpadTapped) {
                // Exit shooting mode if active
                if (shootingState != ShootingState.IDLE) {
                    shootingState = ShootingState.IDLE;
                    isFirstShot = true;
                }
                // Extend outtake OUT (same as init position)
                outtakeServo.setPosition(EXTENDED_POSITION);
            }
            // Note: Left D-pad exit is handled above with left trigger

// ===== Test Servo control (Left Bumper) =====
            // Left bumper: hold to extend fully (1.0), release to retract (0.0)
            boolean leftBumperPressed = gamepad1.left_bumper;
            if (leftBumperPressed) {
                // Left bumper is held - move servo to fully extended position (1.0)
                testServo.setPosition(1.0);
            } else {
                // Left bumper is not held - move servo to fully retracted position (0.0)
                testServo.setPosition(0.0);
            }
            
            // Get battery voltage for telemetry
            VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
            double batteryVoltage = voltageSensor.getVoltage();

            // Spindexer telemetry
            if (yButtonRunning) {
                double runTime = isFirstYPress ? FIRST_Y_RUN_TIME : Y_BUTTON_RUN_TIME;
                telemetry.addData("Spindexer", String.format("Running Y (%.2fs / %.2fs)", yButtonTimer.seconds(), runTime));
            } else {
                telemetry.addData("Spindexer", xButtonPressed ? "Running (X held)" : "Stopped");
            }
            telemetry.addData("Spindexer Angle", "%.2f°", currentAngle);
            telemetry.addData("Spindexer Voltage", "%.3f V", currentVoltage);
            telemetry.addData("First Y Press", isFirstYPress ? "YES" : "NO");
            // Shooting mode status
            String shootingStatus;
            switch (shootingState) {
                case IDLE: shootingStatus = "IDLE (RT to shoot)"; break;
                case CLOSING_OUTTAKE: shootingStatus = "CLOSING OUTTAKE..."; break;
                case SPINNING_UP_FLYWHEELS: shootingStatus = "FLYWHEELS SPINNING UP..."; break;
                case READY_TO_SHOOT: shootingStatus = "READY! (RT to fire)"; break;
                case SHOOTING: shootingStatus = "FIRING!"; break;
                default: shootingStatus = "???"; break;
            }
            telemetry.addData("--- SHOOTING MODE ---", shootingStatus);
            telemetry.addData("Shooting State", shootingState.toString());
            telemetry.addData("Outtake Servo Position", "%.2f", outtakeServo.getPosition());
            telemetry.addData("Test Servo Position", "%.2f", testServo.getPosition());
            telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
            
            // Limelight telemetry (green/purple pipelines)
            telemetry.addData("--- LIMELIGHT ---", "");
            telemetry.addData("Pipeline", "GREEN(0)");
            telemetry.addData("Target Visible", targetVisible ? "YES" : "NO");
            telemetry.addData("tx", "%.1f", llTx);
            telemetry.addData("ty", "%.1f", llTy);
            telemetry.addData("ta", "%.2f", llTa);
            telemetry.addData("age_ms", llAgeMs);
            telemetry.addData("Intake Auto", intakeAutoRunning ? "RUNNING" : "IDLE");

            // Limelight→Pinpoint resync status
            telemetry.addData("--- LIMELIGHT ODOM ---", "");
            if (lastLimelightPose != null) {
                telemetry.addData("LL Pose (in)", "x=%.1f y=%.1f hdg=%.1f",
                        lastLimelightPose.getX(DistanceUnit.INCH),
                        lastLimelightPose.getY(DistanceUnit.INCH),
                        lastLimelightPose.getHeading(AngleUnit.DEGREES));
            } else {
                telemetry.addLine("LL Pose (in): n/a");
            }
            telemetry.addData("Last LL Resync", lastResyncSuccess ? String.format("OK %.2fs ago", lastResyncTimer.seconds()) : "None/failed");
            
            telemetry.update();

            // FTC Dashboard pose packet (Pinpoint)
            TelemetryPacket packet = new TelemetryPacket();
            double px = pinpoint.getPosX(DistanceUnit.INCH);
            double py = pinpoint.getPosY(DistanceUnit.INCH);
            double pHeading = pinpoint.getHeading(AngleUnit.DEGREES);
            packet.put("odom_x_in", px);
            packet.put("odom_y_in", py);
            packet.put("odom_heading_deg", pHeading);
            if (lastLimelightPose != null) {
                packet.put("ll_x_in", lastLimelightPose.getX(DistanceUnit.INCH));
                packet.put("ll_y_in", lastLimelightPose.getY(DistanceUnit.INCH));
                packet.put("ll_heading_deg", lastLimelightPose.getHeading(AngleUnit.DEGREES));
            }
            packet.put("last_resync_success", lastResyncSuccess);
            packet.put("last_resync_age_s", lastResyncTimer.seconds());
            dashboard.sendTelemetryPacket(packet);
        } 
    }
    
    /**
     * Attempt to resync Pinpoint with the latest fresh Limelight AprilTag pose (pipeline 2).
     */
    private void attemptLimelightResync() {
        Pose2D pose = readLimelightAprilTagPose();
        if (pose != null) {
            pinpoint.setPosition(pose);
            lastLimelightPose = pose;
            lastResyncSuccess = true;
            lastResyncTimer.reset();
            seededFromLimelight = true;
        } else {
            lastResyncSuccess = false;
        }
    }

    /**
     * One-time initial seed from Limelight; safe to call once after start.
     */
    private void attemptLimelightInitialSeed() {
        if (seededFromLimelight) {
            return;
        }
        Pose2D pose = readLimelightAprilTagPose();
        if (pose != null) {
            pinpoint.setPosition(pose);
            lastLimelightPose = pose;
            lastResyncSuccess = true;
            lastResyncTimer.reset();
            seededFromLimelight = true;
        } else {
            lastResyncSuccess = false;
        }
    }

    /**
     * Switches to AprilTag pipeline, reads a fresh botpose (guarded by staleness), then restores pipeline.
     */
    private Pose2D readLimelightAprilTagPose() {
        int previousPipeline = getLimelightPipeline();
        Pose2D pose = null;
        try {
            setLimelightPipeline(APRILTAG_PIPELINE);
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                long stalenessMs = result.getStaleness();
                Pose3D botpose = result.getBotpose();
                if (botpose != null && stalenessMs >= 0 && stalenessMs <= MAX_TAG_STALENESS_MS) {
                    pose = toPose2D(botpose);
                }
            }
        } catch (Exception ignored) {
            // Fail quietly
        } finally {
            // Restore pipeline used for vision/auto-intake (green)
            setLimelightPipeline(GREEN_PIPELINE_INDEX);
            if (previousPipeline != GREEN_PIPELINE_INDEX) {
                setLimelightPipeline(previousPipeline);
            }
        }
        return pose;
    }

    /**
     * Convert Limelight botpose (meters + degrees) into Pose2D in inches.
     */
    private Pose2D toPose2D(Pose3D botpose) {
        Position p = botpose.getPosition();
        double xIn = DistanceUnit.METER.toInches(p.x);
        double yIn = DistanceUnit.METER.toInches(p.y);
        double headingDeg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg);
    }
    
    /**
     * Limelight accessors using the FTC SDK Limelight3A class (USB to Control Hub as Ethernet device).
     */
    private int getLimelightPipeline() {
        try {
            LLStatus status = limelight.getStatus();
            return status != null ? status.getPipelineIndex() : desiredPipeline;
        } catch (Exception e) {
            return desiredPipeline;
        }
    }

    private void setLimelightPipeline(int pipelineIndex) {
        try {
            limelight.pipelineSwitch(pipelineIndex);
        } catch (Exception ignored) {
            // Fail quietly to avoid crashing teleop
        }
    }
}