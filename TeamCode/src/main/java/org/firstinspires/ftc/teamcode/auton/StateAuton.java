package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class StateAuton extends OpMode {
    protected int state = 0;
    protected ElapsedTime autonTimer;
    protected ElapsedTime stateTimer;

    protected void initTimers() {
        stateTimer = new ElapsedTime();
        autonTimer = new ElapsedTime();
    }

    protected void advanceState() {
        ++state;
        stateTimer.reset();
    }

    protected void advanceState(int newState) {
        state = newState;
        stateTimer.reset();
    }


}
