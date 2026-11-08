package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.Constants.closedBobbyPos;
import static org.firstinspires.ftc.teamcode.Constants.kF;
import static org.firstinspires.ftc.teamcode.Constants.kP;
import static org.firstinspires.ftc.teamcode.Constants.kSpatulaDownPos;
import static org.firstinspires.ftc.teamcode.Constants.kSpatulaUpPos;
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

@Autonomous(name="UP 2 Tip", group="OCCRA")
public class AutonUp2 extends StateAuton {
    private final RobotState robotState = RobotState.getInstance();
    private MotorEx intake;
    private MotorEx launcher1;
    private MotorEx launcher2;
    private ServoEx gateBobby;
//    private ServoEx spatula;
    private Follower drivetrain;
    private Limelight limelight;
    @Logged private double launcherRpm = 0.0;
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
        autonTimer.reset();
//        spatula.set(kSpatulaUpPos);
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
                    drivetrain.follow(AutonPaths.startToGardenMiddle);
                    targetRpm = 0;
                    intake.set(1.0);
                    gateBobby.set(closedBobbyPos);
                    advanceState();
                }
                break;
            case 3:
                if (!drivetrain.isBusy() || stateTimer.seconds() > 2) {
                    drivetrain.follow(AutonPaths.gardenMiddleToCorner);
                    advanceState();
                }
                break;
            case 4: // Drive to garden middle
                if (stateTimer.seconds() > 3) intake.set(0.0);
                if ((!drivetrain.isBusy() || stateTimer.seconds() > 3) && autonTimer.seconds() > crossTime) {
                    drivetrain.follow(AutonPaths.cornerMiddleCross);
                    targetRpm = 3200;
                    advanceState();
                }
                break;
            case 5: // cross field and go to launch position
                if (stateTimer.seconds() > 1) intake.set(0.0);
                if (!drivetrain.isBusy()) {
                    gateBobby.set(openBobbyPos);
                    intake.set(0);
                    advanceState();
                }
                break;
            case 6: // launch
                if (stateTimer.seconds() > 1) {
                    gateBobby.set(closedBobbyPos);
//                    drivetrain.follow(AutonPaths.farFlowerIntake);
                    drivetrain.follow(AutonPaths.farScoreToPark);
//                    spatula.set(kSpatulaDownPos);
                    intake.set(1.0);
                    targetRpm = 0;
                    advanceState();
                }
                break;
            case 7:
                if (!drivetrain.isBusy()) {
                    advanceState(20);
                }
                break;
//            case 7: // Drive until in flower
//                if (!drivetrain.isBusy()) {
//                    advanceState();
//                }
//                break;
//            case 8: // Wait to let pollen roll into robot
//                if (stateTimer.seconds() > 3) {
//                    drivetrain.follow(AutonPaths.farFlowerToScore);
//                    advanceState();
//                }
//                break;
//            case 9:
//                if (stateTimer.seconds() > 0.4) {
//                    spatula.set(kSpatulaUpPos);
//                }
//                break;
            case 20:
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
