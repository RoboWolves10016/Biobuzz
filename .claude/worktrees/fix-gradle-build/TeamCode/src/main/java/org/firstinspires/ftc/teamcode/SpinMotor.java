package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.robowolves.wolfkit.Logged;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name="Launcher Test")
public class SpinMotor extends OpMode {
    MotorEx motor1;
    MotorEx motor2;
    double power = 0;

    @Logged double rpm1 = 0;
    @Logged double rpm2 = 0;

    @Override
    public void init() {
        motor1 = new MotorEx(hardwareMap,"Launcher1", Motor.GoBILDA.BARE);
        motor1.setRunMode(Motor.RunMode.RawPower);

        motor2 = new MotorEx(hardwareMap, "Launcher2", Motor.GoBILDA.BARE);
        motor2.setRunMode(Motor.RunMode.RawPower);
    }

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) power = power + 0.1;
        if (gamepad1.leftBumperWasPressed()) power = power - 0.1;

        if (power > 1) power = 1;
        if (power < -1) power = -1;

        motor1.set(power);
        motor2.set(-power);

        rpm1 = motor1.getCorrectedVelocity();
        rpm2 = motor2.getCorrectedVelocity();

        telemetry.addData("Power Output", power);
        telemetry.addData("RPM 1", rpm1);
        telemetry.addData("RPM 2", rpm2);
    }
}
