package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Method;

@TeleOp(name = "Pinpoint Quick Config (one-time)", group = "Config")
public class PinpointQuickConfig extends LinearOpMode {

    // ---- CHANGE THESE ONCE ----
    private static final String PINPOINT_NAME = "pinpoint";

    // Your swingarm wheel diameter
    private static final double WHEEL_DIAMETER_MM = 48.0;

    // Pod offsets from robot center (measure and update!)
    // Convention used by many Pinpoint examples:
    //  - X pod lateral offset: left of center +, right of center -
    //  - Y pod forward offset: forward of center +, back of center -
    private static final double X_POD_LATERAL_OFFSET_IN = 6.0;
    private static final double Y_POD_FORWARD_OFFSET_IN = 6.7;

    // Encoder directions (set these how you want your signs)
    // If forward makes X negative, flip X direction.
    // If left makes Y negative (but you want left positive), flip Y direction.
    private static final GoBildaPinpointDriver.EncoderDirection X_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    private static final GoBildaPinpointDriver.EncoderDirection Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    @Override
    public void runOpMode() {

        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        telemetry.addLine("Pinpoint Quick Config");
        telemetry.addLine("This will try to set wheel diameter + offsets, then reset.");
        telemetry.addLine("Run once, then use your normal OpModes.");
        telemetry.update();

        waitForStart();

        // 1) Directions (you confirmed this method exists)
        boolean directionsOK = safeCall(
                pinpoint,
                new String[]{"setEncoderDirections"},
                new Class<?>[]{GoBildaPinpointDriver.EncoderDirection.class, GoBildaPinpointDriver.EncoderDirection.class},
                new Object[]{X_DIR, Y_DIR}
        );

        // 2) Wheel diameter (your driver DID NOT have setOdometryWheelDiameterMM, so we try common variants)
        boolean wheelOK =
                safeCall(pinpoint, new String[]{"setOdometryWheelDiameterMM"}, new Class<?>[]{double.class}, new Object[]{WHEEL_DIAMETER_MM})
                        || safeCall(pinpoint, new String[]{"setOdometryWheelDiameterMm"}, new Class<?>[]{double.class}, new Object[]{WHEEL_DIAMETER_MM})
                        || safeCall(pinpoint, new String[]{"setWheelDiameterMM"}, new Class<?>[]{double.class}, new Object[]{WHEEL_DIAMETER_MM})
                        || safeCall(pinpoint, new String[]{"setWheelDiameterMm"}, new Class<?>[]{double.class}, new Object[]{WHEEL_DIAMETER_MM})
                        || safeCall(pinpoint, new String[]{"setWheelDiameter"}, new Class<?>[]{double.class}, new Object[]{WHEEL_DIAMETER_MM});

        // 3) Pod offsets (your driver DID NOT have setPodOffsets, so we try common variants)
        // Try offsets in inches with a DistanceUnit, then without a unit.
        boolean offsetsOK =
                safeCall(pinpoint,
                        new String[]{"setPodOffsets", "setOffsets"},
                        new Class<?>[]{double.class, double.class, DistanceUnit.class},
                        new Object[]{X_POD_LATERAL_OFFSET_IN, Y_POD_FORWARD_OFFSET_IN, DistanceUnit.INCH})
                        || safeCall(pinpoint,
                        new String[]{"setPodOffsets", "setOffsets"},
                        new Class<?>[]{double.class, double.class},
                        new Object[]{X_POD_LATERAL_OFFSET_IN, Y_POD_FORWARD_OFFSET_IN});

        // 4) Reset after config (you confirmed this exists)
        boolean resetOK = safeCall(
                pinpoint,
                new String[]{"resetPosAndIMU"},
                new Class<?>[]{},
                new Object[]{}
        );

        telemetry.clear();
        telemetry.addLine("=== Pinpoint Quick Config Results ===");
        telemetry.addData("Directions set", directionsOK ? "YES" : "NO");
        telemetry.addData("Wheel diameter set", wheelOK ? "YES" : "NO");
        telemetry.addData("Pod offsets set", offsetsOK ? "YES" : "NO");
        telemetry.addData("Reset Pos+IMU", resetOK ? "YES" : "NO");
        telemetry.addLine("");
        telemetry.addLine("If Wheel/Offsets say NO, your driver API can't set them.");
        telemetry.addLine("That means they must be set via goBILDA config tool / newer library.");
        telemetry.update();

        // Sit here so you can read telemetry
        while (opModeIsActive()) {
            sleep(50);
        }
    }

    /**
     * Tries to call the first method name that exists with the given signature.
     * Returns true if invoked successfully.
     */
    private boolean safeCall(Object target,
                             String[] methodNames,
                             Class<?>[] paramTypes,
                             Object[] args) {
        for (String name : methodNames) {
            try {
                Method m = target.getClass().getMethod(name, paramTypes);
                m.invoke(target, args);
                return true;
            } catch (NoSuchMethodException e) {
                // try next name
            } catch (Exception e) {
                telemetry.addData("Call failed", name + ": " + e.getClass().getSimpleName());
                telemetry.update();
                return false;
            }
        }
        return false;
    }

    private boolean safeCall(Object target,
                             String methodName,
                             Class<?>[] paramTypes,
                             Object[] args) {
        return safeCall(target, new String[]{methodName}, paramTypes, args);
    }
}