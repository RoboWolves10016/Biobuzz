package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class RobotState {
    private static RobotState instance = null;
    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private Pose robotPose = Pose.zero();
    public void setPose(Pose pose) {
        robotPose = pose;
    }
    public Pose getPose() {
        return robotPose;
    }
}
