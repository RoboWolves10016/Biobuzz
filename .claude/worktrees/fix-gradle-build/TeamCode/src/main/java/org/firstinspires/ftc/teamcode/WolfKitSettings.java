package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.robowolves.wolfkit.WolfKitConfig;

import org.firstinspires.ftc.ftccommon.external.OnCreate;

/**
 * Settings for the features WolfKit performs automatically.
 *
 * <p>The SDK calls {@link #configure(Context)} once when the robot controller app starts,
 * before any OpMode runs. Each setting below is shown with its default value.
 *
 * <p>These settings do not affect explicit calls such as {@code WolfKit.log}, which always work.
 */
public class WolfKitSettings {
    /**
     * Applies the settings.
     *
     * @param context the application context
     */
    @OnCreate
    public static void configure(Context context) {
        // Record a WPILOG file for each OpMode run, from init to stop, in WolfKit/logs on the
        // robot controller's storage. When false, values are still streamed to connected
        // dashboards but are not kept.
        WolfKitConfig.recordLogFiles = true;

        // The number of log files to keep. When a new run would exceed it, the oldest files
        // are deleted. A log renamed from the dashboard is not counted or deleted.
        WolfKitConfig.keepLogFiles = 15;

        // Log every stick, trigger and button of both gamepads under Gamepad1/ and Gamepad2/
        // while an OpMode is active. A background thread samples the gamepads, so samples are
        // not synchronised with the robot loop.
        WolfKitConfig.logGamepads = true;

        // The interval between gamepad samples, in milliseconds. A button press shorter than
        // the interval may not be logged.
        WolfKitConfig.gamepadIntervalMs = 20;

        // Log the battery voltage under WolfKit/BatteryVolts. Each reading is a command to the
        // hub of 2 to 3 ms on the bus the robot loop uses, and may delay one loop iteration.
        WolfKitConfig.logBattery = true;

        // The interval between battery voltage readings, in seconds.
        WolfKitConfig.batteryIntervalSeconds = 10;

        // Publish the text OpModes send to the Driver Station through telemetry, at most ten
        // times a second, for the dashboard's Telemetry widget. WolfKit replaces the OpMode's
        // telemetry with a pass-through before init; every call still reaches the Driver
        // Station. When false, logAnnotatedOnTelemetryUpdate has no effect.
        WolfKitConfig.captureTelemetry = true;

        // Register each OpMode before its init, so that @Logged members of the OpMode, and of
        // objects reachable through them, are found without calling WolfKit.register.
        WolfKitConfig.registerOpModes = true;

        // Log all @Logged members each time an OpMode updates its telemetry. The SDK updates
        // telemetry after every loop() of an iterative OpMode; a LinearOpMode is covered
        // whenever it calls telemetry.update(). An OpMode that calls WolfKit.logAll() itself
        // is exempt, so that members are not logged twice.
        WolfKitConfig.logAnnotatedOnTelemetryUpdate = true;

        // Log the period of the robot loop under WolfKit/LoopMs, in milliseconds.
        WolfKitConfig.logLoopTime = true;
    }
}
