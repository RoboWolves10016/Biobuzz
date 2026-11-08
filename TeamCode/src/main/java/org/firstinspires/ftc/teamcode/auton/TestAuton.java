package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.Constants.closedBobbyPos;
import static org.firstinspires.ftc.teamcode.Constants.openBobbyPos;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.robowolves.wolfkit.Logged;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Disabled
@Autonomous(name="Test Auton")
public class TestAuton extends StateAuton{

    private final RobotState robotState = RobotState.getInstance();
    private MotorEx intake;
    private MotorEx launcher1;
    private MotorEx launcher2;
    private ServoEx gateBobby;
    //    private ServoEx spatula;
    private Follower drivetrain;
    private Limelight limelight;
    @Logged
    private double launcherRpm = 0.0;
    @Logged private double targetRpm = 0.0;
    int crossTime = 6;

    @Override
    public void init() {
        initTimers();
        drivetrain = Constants.create(hardwareMap);

        launcher1 = new MotorEx(hardwareMap, "Launcher1", Motor.GoBILDA.BARE);
        launcher1.setRunMode(Motor.RunMode.RawPower);
        launcher1.setInverted(true);

        launcher2 = new MotorEx(hardwareMap, "Launcher2", Motor.GoBILDA.BARE);
        launcher2.setRunMode(Motor.RunMode.RawPower);
        launcher2.setInverted(false);

        intake = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.BARE);
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setInverted(true);

        gateBobby = new ServoEx(hardwareMap, "Bobby");
//        spatula = new ServoEx(hardwareMap, "spatula");

        limelight = new Limelight(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Press A to increase garden leave time, B to decrease");
        telemetry.addData("Configured Garden Leave Time (s)", crossTime);
        if (gamepad1.aWasPressed()) ++crossTime;
        if (gamepad1.bWasPressed()) --crossTime;
    }

    @Override
    public void start() {
        drivetrain.setPose(AutonPaths.upStart);
        stateTimer.reset();
        initTimers();
    }


    @Override
    public void loop() {
//        switch (state) {
//            case 0:
//                gateBobby.set(closedBobbyPos);
//                targetRpm = 3200;
//                advanceState();
//                break;
//            case 1:
//                if (Math.abs(launcherRpm - targetRpm) < 50) advanceState();
//                break;
//        }
    }
}
