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
        c.xPodOffset.set(1.3152014364407758);
        c.yPodOffset.set(-4.463954835426151);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.23846459920099697);
                Controller secondaryTranslationalForward = Controller.proportional(0.08810629751332492);
                Controller primaryTranslationalLateral = Controller.proportional(0.4045830879129142);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14948264032460998);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017254818636828963));
                c.brake.set(Controller.proportionalFeedforward(0.014666595841304617));

                c.headingFeedback.set(Controller.proportional(3.92789763245283));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04643459119492776, 0.004356751113233625));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07226709362999605, 0.06266078320083113));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010934756898756301, 0.0013555737598267407));

                c.maxAchievableForwardVelocity.set(60.78274270104115);
                c.maxAchievableStrafeVelocity.set(50.77943233315055);
                c.naturalForwardDeceleration.set(33.56698917706593);
                c.naturalStrafeDeceleration.set(64.75897979864621);
            }
    );

    public static Follower create(HardwareMap h) {
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
        return new Follower(
                createFusionLocalizer(h),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}