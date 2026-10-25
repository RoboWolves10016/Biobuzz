package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.robowolves.wolfkit.Logged;
import com.robowolves.wolfkit.WolfKit;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name="Launcher Test")
public class SpinLauncher extends OpMode {
    MotorEx motor1;
    MotorEx motor2;
    boolean on = false;
    @Logged double targetRpm = 0;

    @Logged double rpm1 = 0;
    @Logged double rpm2 = 0;

    @Logged double kF = 0.13;
    @Logged double kV = 0.00019;

    @Logged double kP = 0.005;

//    private final PIDController speedController = new PIDController(kP, 0, 0);

    @Override
    public void init() {
        motor1 = new MotorEx(hardwareMap,"Launcher1", Motor.GoBILDA.BARE);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor1.setInverted(true);

        motor2 = new MotorEx(hardwareMap, "Launcher2", Motor.GoBILDA.BARE);
        motor2.setRunMode(Motor.RunMode.RawPower);
        motor2.setInverted(false);
    }

    @Override
    public void loop() {
        kV = WolfKit.tunable("kV", 0.002);
        kP = WolfKit.tunable("kP", 0);
        kF = WolfKit.tunable("kF", 0.0);

        if (gamepad1.rightBumperWasPressed()) targetRpm = targetRpm + 100;
        if (gamepad1.leftBumperWasPressed()) targetRpm = targetRpm - 100;
//
//        if (gamepad1.rightBumperWasPressed()) power = power + 0.1;
//        if (gamepad1.leftBumperWasPressed()) power = power - 0.1;

//        if (power > 1) power = 1;
//        if (power < -1) power = -1;

        rpm1 = (motor1.getCorrectedVelocity() / 28) * 60;
        rpm2 = (motor2.getCorrectedVelocity() / 28) * 60;

//        double error = rpm1 - targetRpm;

        double power = kV * targetRpm;
        double error = targetRpm - rpm1;
        power += kP * error;
        power += kF * Math.signum(targetRpm);

        motor1.set(power);
        motor2.set(power);

        telemetry.addData("Target Speed", targetRpm);
        telemetry.addData("RPM 1", rpm1);
        telemetry.addData("RPM 2", rpm2);
    }
}
