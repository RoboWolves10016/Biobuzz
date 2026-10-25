package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.robowolves.wolfkit.Logged;
import com.robowolves.wolfkit.WolfKit;

/**
 * Demonstrates WolfKit logging and tuning. Requires no hardware.
 *
 * <p>Run it and open {@code http://192.168.43.1:8080/dash} to see the values live.
 */
@TeleOp(name = "WolfKit Demo", group = "Test")
public class WolfKitDemo extends OpMode {

    @Logged private double annotationTestVariable = 0;

    @Override
    public void init() {}

    @Override
    public void loop() {
        double t = getRuntime();
        double speed = WolfKit.tunable("Demo/Speed", 1.0);

        annotationTestVariable = Math.sin(t * speed);

        WolfKit.log("Demo/Sine", Math.sin(t * speed));
        WolfKit.log("Demo/LeftStickY", -gamepad1.left_stick_y);
        WolfKit.log("Demo/A", gamepad1.a);
        WolfKit.log("Demo/State", gamepad1.a ? "PRESSED" : "IDLE");

        // A pose moving on a circle of radius 24 inches about the field centre, in Pedro coordinates.
        WolfKit.logPedroPose("Demo/Robot", 72 + 24 * Math.cos(t * speed), 72 + 24 * Math.sin(t * speed), t * speed + Math.PI / 2);
    }
}
