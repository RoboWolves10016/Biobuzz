package org.firstinspires.ftc.teamcode.auton;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import static org.firstinspires.ftc.teamcode.pedro.Constants.foresightConfig;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

public class AutonPaths {
    private static final PoseFactory poseFactory = PoseFactory.degrees();

    // UP SIDE POSES
    public static final Pose upStart = poseFactory.of(54.75, 10.5, 270);
    private static final Pose upStartParkControl = poseFactory.of(15.75, 28.75, 0);
    public static final Pose upPark = poseFactory.of(18, 93, 270);
    private static final Pose upStartOffWall = poseFactory.of(54.75, 20, 225);
    public static final Pose gardenMiddle = poseFactory.of(30, 14, 180);
    public static final Pose gardenCorner = poseFactory.of(14, 14, 180);
    public static final Pose crossFieldMidpoint = poseFactory.of(70, 27, 90);
    public static final Pose farScoreUnder = poseFactory.of(70, 130, 90);
    public static final Pose farScoreParkControl = poseFactory.of(42, 95, 0);
    public static final Pose farScorePark = poseFactory.of(22, 90, 0);
//    public static final Pose gardenToScoreMiddle = poseFactory.of(58, )
//    public static final Pose farPreFlower = poseFactory.of(56,120, 90);
//    public static final Pose farFlower = poseFactory.of(56, 130, 90);




    // DOWN SIDE POSES
    public static final Pose downStart = poseFactory.of(62.75, 133.75, 90);
    private static final Pose downStartParkControl = poseFactory.of(62.75, 117.75, 0);
    public static final Pose downPark = poseFactory.of(11.75, 125, 90);

    public static Path upStartToPark = curve(upStart, upStartParkControl, upPark)
            .constant(upPark);
    public static Path downStartToPark = curve(downStart, downStartParkControl, downPark)
            .constant(downPark);

    public static Path startToGardenMiddle = path(
            line(upStart, upStartOffWall).linear(upStart, upStartOffWall),
            line(upStartOffWall, gardenMiddle).constant(gardenMiddle));
    public static Path gardenMiddleToCorner = line(gardenMiddle, gardenCorner).linear(gardenMiddle, gardenCorner)
            .with(foresightConfig.maxVelocityConstraint.at(10.0));


    public static Path cornerMiddleCross = path(
            line(gardenCorner,crossFieldMidpoint).heading(
                    Interpolator.piecewise()
                            .until(0.6, Interpolator.tangent.reverse())
                            .until(1.0, Interpolator.linear(Math.toRadians(-160), crossFieldMidpoint.heading()))),
            line(crossFieldMidpoint, farScoreUnder).tangent());

//    public static final Path cornerToScore = path(
//            line(gardenCorner,)
//    )

//    public static final Path farFlowerIntake = path(
//            line(farScoreUnder, farPreFlower).linear(farScoreUnder, farPreFlower),
//            line(farPreFlower, farFlower).linear(farPreFlower, farFlower)
//                    .with(foresightConfig.maxVelocityConstraint.at(20.0)));
//
//    public static final Path farFlowerToScore = path(
//            line(farFlower, farPreFlower).linear(farFlower, farPreFlower),
//            line(farPreFlower, farScoreUnder).linear(farPreFlower, farScoreUnder)
//    );
    public static Path farScoreToPark = curve(farScoreUnder, farScoreParkControl, farScorePark).reverseTangent();

    // An empty path() throws when this class loads, which crashes any auton that uses AutonPaths.
    // Uncomment once it has at least one segment.
//    public static final Path farScoreToCloseFlower = path(
//
//    );

}
