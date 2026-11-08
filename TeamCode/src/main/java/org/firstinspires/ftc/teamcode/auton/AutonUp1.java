package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.Constants.closedBobbyPos;
import static org.firstinspires.ftc.teamcode.Constants.kF;
import static org.firstinspires.ftc.teamcode.Constants.kP;
import static org.firstinspires.ftc.teamcode.Constants.kV;
import static org.firstinspires.ftc.teamcode.Constants.openBobbyPos;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.robowolves.wolfkit.Logged;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Autonomous(name="UP 1 Tip", group="OCCRA")
public class AutonUp1 extends StateAuton {
    private final RobotState robotState = RobotState.getInstance();
    private MotorEx intake;
    private MotorEx launcher1;
    private MotorEx launcher2;
    private ServoEx gateBobby;
    private Follower drivetrain;
    private Limelight limelight;
    @Logged private double launcherRpm = 0.0;
    @Logged private double targetRpm = 0.0;

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

        limelight = new Limelight(hardwareMap);
    }

    @Override
    public void start() {
        drivetrain.setPose(AutonPaths.upStart);
        stateTimer.reset();
    }

    @Override
    public void loop() {
        switch (state) {
            case 0:
                gateBobby.set(closedBobbyPos);
                targetRpm = 3200;
                advanceState();
                break;
            case 1:
                if (Math.abs(launcherRpm - targetRpm) < 50) advanceState();
                break;
            case 2:
                gateBobby.set(openBobbyPos);
                if (stateTimer.seconds() > 1) {
                    advanceState();
                    drivetrain.follow(AutonPaths.upStartToPark);
                    targetRpm = 0;
                }
                break;
            case 3:
                if (!drivetrain.isBusy()) advanceState();
                break;
            case 4:
                stop();
                break;
        }

        loopLauncher();
        drivetrain.update();
        robotState.setPose(drivetrain.pose());
    }

    private void loopLauncher() {
        launcherRpm = (launcher1.getCorrectedVelocity() / 28) * 60;
        double power = kV * targetRpm;
        double error = targetRpm - launcherRpm;
        power += kP * error;
        power += kF * Math.signum(targetRpm);

        launcher1.set(power);
        launcher2.set(power);
    }
}
