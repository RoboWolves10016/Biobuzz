package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.closedBobbyPos;
import static org.firstinspires.ftc.teamcode.Constants.kF;
import static org.firstinspires.ftc.teamcode.Constants.kP;
import static org.firstinspires.ftc.teamcode.Constants.kV;
import static org.firstinspires.ftc.teamcode.Constants.openBobbyPos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.robowolves.wolfkit.Logged;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name="Auton UP Side", group="OCCRA")
public class AutonUp extends OpMode {
    private final RobotState robotState = RobotState.getInstance();
    private MotorEx intake;
    private MotorEx launcher1;
    private MotorEx launcher2;
    private ServoEx gateBobby;
    private Follower drivetrain;
    @Logged private double launcherRpm = 0.0;
    @Logged private double targetRpm = 0.0;

    private int state = 0;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.75, 7.75, 270);
    private final Pose path1 = poseFactory.of(13.75, 89, 270);
    private final Pose path1Control1 = poseFactory.of(15.75, 28.75, 0);

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).constant(path1);
    }


    @Override
    public void init() {
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
    }

    @Override
    public void start() {
        drivetrain.setPose(start);
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
                    drivetrain.follow(path1());
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

    private void advanceState() {
        ++state;
        stateTimer.reset();
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
