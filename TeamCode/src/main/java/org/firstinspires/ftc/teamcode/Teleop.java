package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.closedBobbyPos;
import static org.firstinspires.ftc.teamcode.Constants.kCloseLeftZeroPose;
import static org.firstinspires.ftc.teamcode.Constants.kCloseRightZeroPose;
import static org.firstinspires.ftc.teamcode.Constants.kF;
import static org.firstinspires.ftc.teamcode.Constants.kFarLeftZeroPose;
import static org.firstinspires.ftc.teamcode.Constants.kFarRightZeroPose;
import static org.firstinspires.ftc.teamcode.Constants.kLeftCellPose;
import static org.firstinspires.ftc.teamcode.Constants.kP;
import static org.firstinspires.ftc.teamcode.Constants.kRightCellPose;
import static org.firstinspires.ftc.teamcode.Constants.kSpatulaUpPos;
import static org.firstinspires.ftc.teamcode.Constants.kV;
import static org.firstinspires.ftc.teamcode.Constants.openBobbyPos;
import static org.firstinspires.ftc.teamcode.pedro.Constants.createFused;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.robowolves.wolfkit.Logged;
import com.robowolves.wolfkit.WolfKit;
import com.robowolves.wolfkit.pedro.WolfKitPedro;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;


import org.firstinspires.ftc.teamcode.Constants;


import java.util.List;

@TeleOp(name="Teleop", group="OCCRA")
public class Teleop extends OpMode {
    RobotState robotState = RobotState.getInstance();
    private Follower drivetrain;
    private MotorEx intake;
    private MotorEx launcher1;
    private MotorEx launcher2;
    private ServoEx gateBobby;

    private ServoEx spatula;
    private List<LynxModule> hubs;


    private PIDController aimController = new PIDController(0, 0, 0);
//    private Apiary apiary;
//    private ApiaryPedro vision;

    @Logged
    double targetRpm = 0.0;
    double lastTargetRpm = 0.0;
    @Logged
    double launcherRpm = 0.0;

    @Logged boolean autoAim = false;
    @Logged double spatulaTarget = kSpatulaUpPos;

    @Override
    public void init() {
        // Read every motor encoder on a hub in one command per loop instead of one per call.
        // MANUAL means the cache holds until clearBulkCache(), which loop() does first thing.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        drivetrain = createFused(hardwareMap);
        drivetrain.manual();
//        vision = new ApiaryPedro((FusionLocalizer) drivetrain.localizer);

        launcher1 = new MotorEx(hardwareMap, "Launcher1", Motor.GoBILDA.BARE);
        launcher1.setRunMode(Motor.RunMode.RawPower);
        launcher1.setInverted(true);

        launcher2 = new MotorEx(hardwareMap, "Launcher2", Motor.GoBILDA.BARE);
        launcher2.setRunMode(Motor.RunMode.RawPower);
        launcher2.setInverted(false);

        intake = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.BARE);
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setInverted(true);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        gateBobby = new ServoEx(hardwareMap, "Bobby");

        spatula = new ServoEx(hardwareMap, "spatula");
        spatula.setCachingTolerance(0.01);
//        apiary = new Apiary(
//                hardwareMap,
//                "limelight",
//                cameraForward,
//                cameraLeft,
//                cameraUp,
//                cameraTiltUp,
//                180,
//                0);
    }

    @Override
    public void start() {
        drivetrain.setPose(robotState.getPose());
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        autoAim = gamepad1.left_bumper;
//        kV = WolfKit.tunable("kV", 0.00019);
//        kF = WolfKit.tunable("kF", 0.13);
//        kP = WolfKit.tunable("kP", 0.002);
        boolean launcherOverride = WolfKit.tunable("LauncherOverride", false);
        if (launcherOverride) targetRpm = WolfKit.tunable("TargetRPM", 0);


//        if (gamepad2.rightBumperWasPressed()) targetRpm = targetRpm + 100;
//        if (gamepad2.leftBumperWasPressed()) targetRpm = targetRpm - 100;
        if (targetRpm == 1800) {
            ;
        } else if (gamepad2.y) {
            targetRpm = 3225;
        } else if (gamepad2.b) {
            targetRpm = 0.0;
        }

        if (gamepad2.rightBumperWasPressed()) {
            lastTargetRpm = targetRpm;
            targetRpm = 1800;
        }
        if (gamepad2.rightBumperWasReleased()) {
            targetRpm = lastTargetRpm;
        }

        launcherRpm = (launcher1.getCorrectedVelocity() / 28) * 60;

        double power = kV * targetRpm;
        double error = targetRpm - launcherRpm;
        power += kP * error;
        power += kF * Math.signum(targetRpm);

        launcher1.set(power);
        launcher2.set(power);

        double intakePower = WolfKit.tunable("IntakePower", 0);
        if (gamepad2.left_trigger > 0.1) {
            intakePower = 1;
        } else if (gamepad2.left_bumper) {
            intakePower = -1;
        }
        intake.set(intakePower);


        boolean wasAlreadyOpen = gateBobby.get() == openBobbyPos;
        double gateTarget = closedBobbyPos;
        if ((gamepad2.right_bumper || gamepad2.right_trigger > 0.1)
                && targetRpm != 0
                && (Math.abs(targetRpm - launcherRpm) < 50
                || wasAlreadyOpen)
        ) {
            gateTarget = openBobbyPos;
        }

        boolean servoOverride = WolfKit.tunable("ServoOverride", false);
        if (servoOverride) gateTarget = WolfKit.tunable("ServoTarget", openBobbyPos);
        gateBobby.set(gateTarget);

        double turn = -gamepad1.right_stick_x;
        aimController.setPID(
                WolfKit.tunable("aimKP", 1),
                WolfKit.tunable("aimKI", 0.01),
                WolfKit.tunable("aimKD", 0.2));

        Pose goalPose = drivetrain.pose().y() > 72 ? kLeftCellPose : kRightCellPose;
        Vector2D vectorToGoal = goalPose.minus(drivetrain.pose()).toVector2D();
        if (autoAim) {
            double headingError = vectorToGoal.theta() - drivetrain.pose().heading() + Math.PI;
            if (headingError > Math.PI) headingError -= 2 * Math.PI;
            turn = aimController.calculate(0, headingError);
        }

        DrivePowers fieldCentric = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turn,
                drivetrain.pose().heading());

        if (gamepad1.left_trigger_pressed) {
            drivetrain.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        } else {
            drivetrain.manual(fieldCentric);
        }

        drivetrain.update();

        WolfKitPedro.log("Drive/Fused", drivetrain.pose());

        if (gamepad1.back) {
            Pose zeroPose = drivetrain.pose().x() < 72 ? (
                    drivetrain.pose().y() > 72 ? kCloseLeftZeroPose : kCloseRightZeroPose
            ) : (
                    drivetrain.pose().y() > 72 ? kFarLeftZeroPose : kFarRightZeroPose
            );
            drivetrain.setPose(zeroPose);
        }
        robotState.setPose(drivetrain.pose());

        if (gamepad1.rightBumperWasPressed()) {
            if (spatulaTarget == Constants.kSpatulaUpPos) spatulaTarget = Constants.kSpatulaDownPos;
            else spatulaTarget = Constants.kSpatulaUpPos;
        }

        if (WolfKit.tunable("SpatulaOverride", false)) {
            spatulaTarget = WolfKit.tunable("SpatulaOverridePos", 0.5);
        }
        spatula.set(spatulaTarget);
    }

    @Override
    public void stop() {
//        apiary.stop();
    }
}
