package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
    c.frontLeftName.set("fl");
    c.frontRightName.set("fr");
    c.backLeftName.set("bl");
    c.backRightName.set("br");
    c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
    c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
    c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
    c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

    c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(56 / 25.4);
        c.yPodOffset.set(-5.625);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2605077427560874);
                Controller secondaryTranslationalForward = Controller.proportional(0.0962506500532872);
                Controller primaryTranslationalLateral = Controller.proportional(0.3718985657670391);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13740658273822554);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016960121013997943));
                c.brake.set(Controller.proportionalFeedforward(0.01441610286189825));

                c.headingFeedback.set(Controller.proportional(5.039041100804522));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03777735444857529, 0.007490942697974316));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0658940804125789, 0.045542338125887025));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001273330955467476, 0.0016376365397046892));

                c.maxAchievableForwardVelocity.set(63.07768261795754);
                c.maxAchievableStrafeVelocity.set(50.1385458752296);
                c.naturalForwardDeceleration.set(29.996704795650253);
                c.naturalStrafeDeceleration.set(66.79610029977103);
            }
    );

    /**
     * Caps path speed at a fraction of the robot's top speed, from the next loop on. Call it just
     * before {@code follow()} to slow that path, and {@code setSpeed(1)} to go back to full speed.
     * Every follower starts at full speed.
     *
     * @param fraction 0.5 for half speed; 1 or more for no cap
     */
    public static void setSpeed(double fraction) {
        if (!(fraction > 0)) throw new IllegalArgumentException("Speed must be above 0, not " + fraction);
        foresightConfig.maxPathSpeed.set(fraction >= 1 ? ForesightConfig.Constraint.NONE : fraction);
    }

    public static Follower create(HardwareMap h) {
        // The config is shared by every OpMode, so a cap set by one must not carry into the next.
        setSpeed(1);
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    // Kalman filter over the Pinpoint, for fusing vision positions. Variances are in in² and rad².
    // How sure the filter is of the starting pose: 3 in, 2 degrees.
    public static Pose fusionInitialVariance = new Pose(9, 9, Math.toRadians(2) * Math.toRadians(2));
    // Variance the Pinpoint gains per inch or radian driven: about 1 in of drift per 100 in.
    public static Pose fusionProcessVariance = new Pose(0.01, 0.01, 1e-4);
    // Used only for measurements added without their own variance; Apiary always gives its own.
    public static Pose fusionMeasurementVariance = new Pose(4, 4, 1);
    // Loops of history kept, so that a camera frame can be applied at the time it was captured.
    public static int fusionHistoryLength = 200;

    /** The Pinpoint wrapped in a Kalman filter that vision positions can be added to. */
    public static FusionLocalizer createFusionLocalizer(HardwareMap h) {
        return new FusionLocalizer(
                new PinpointLocalizer(h, localizerConfig),
                fusionInitialVariance,
                fusionProcessVariance,
                fusionMeasurementVariance,
                fusionHistoryLength);
    }

    /**
     * A follower that steers on the fused pose. Add vision positions to its filter, which is
     * {@code (FusionLocalizer) follower.localizer}.
     */
    public static Follower createFused(HardwareMap h) {
        setSpeed(1);
        return new Follower(
                createFusionLocalizer(h),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}