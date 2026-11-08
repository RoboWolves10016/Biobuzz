package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.HiveState;
import org.firstinspires.ftc.teamcode.RobotState;

import java.util.ArrayList;
import java.util.List;

public class Limelight extends Subsystem {
    RobotState robotState;
    private final HardwareMap hardwareMap;
    private Limelight3A limelight;

    private LLResult latestResult;
    private List<Integer> visibleTags;
    private HiveState redState;
    private HiveState blueState;

    public Limelight(HardwareMap hwMap) {
        robotState = RobotState.getInstance();
        hardwareMap = hwMap;

        visibleTags = new ArrayList<>();
    }

    @Override
    void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    @Override
    void run() {
        latestResult = limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> tags = latestResult.getFiducialResults();
        List<Integer> tagIds = new ArrayList<>();
        for (int i = 0; i < tags.size(); ++i) {
            tagIds.add(tags.get(i).getFiducialId());
        }
        visibleTags = tagIds;

    }

    @Override
    protected void updateTelemetry() {

    }

    @Override
    void stop() {

    }
    public boolean seesRedLeft() {
        for (int i = 0; i < visibleTags.size(); ++i) {
            int id = visibleTags.get(i);
            if (id >= 30 && id < 34) return true;
        }
        return false;
    }

    public boolean seesRedRight() {
        for (int i = 0; i < visibleTags.size(); ++i) {
            int id = visibleTags.get(i);
            if (id >= 34 && id < 38) return true;
        }
        return false;
    }

    public boolean seesBlueLeft() {
        for (int i = 0; i < visibleTags.size(); ++i) {
            int id = visibleTags.get(i);
            if (id >= 38 && id < 42) return true;
        }
        return false;
    }

    public boolean seesBlueRight() {
        for (int i = 0; i < visibleTags.size(); ++i) {
            int id = visibleTags.get(i);
            if (id >= 42 && id < 46) return true;
        }
        return false;
    }

    public boolean seesLeft(Alliance alliance) {
        if (alliance == Alliance.RED) return seesRedLeft();
        return seesBlueLeft();
    }

    public boolean seesRight(Alliance alliance) {
        if (alliance == Alliance.RED) return seesRedRight();
        return seesBlueRight();
    }

}
