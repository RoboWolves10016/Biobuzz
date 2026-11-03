package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class Constants {

    public static final double kF = 0.13;
    public static final double kP = 0.002;
    public static final double kV = 0.00019;
    public static final double openBobbyPos = 0.6;
    public static final double closedBobbyPos = 0.84;

    // Limelight lens position from the robot's centre, for Apiary. Placeholders: measure these.
    public static final double cameraForward = -2;   // inches
    public static final double cameraLeft = 0.5;      // inches
    public static final double cameraUp = 13.75;       // inches above the floor
    public static final double cameraTiltUp = 30.0;   // degrees up from level

    public static final Pose kRightCellPose = new Pose(57.5, 50);
    public static final Pose kLeftCellPose = new Pose(57.5, 94);

    public static final Pose kCloseLeftZeroPose = new Pose(10.125, 144-7.87, Math.PI);
    public static final Pose kCloseRightZeroPose = new Pose(10.125, 7.87, Math.PI);
    public static final Pose kFarLeftZeroPose = new Pose(144-7.875, 144-7.87, Math.PI);
    public static final Pose kFarRightZeroPose = new Pose(144-7.875, 7.87, Math.PI);

}
