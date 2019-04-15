package com._604robotics.robot2019.macros;

import com._604robotics.robot2019.modules.Drive;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;

public abstract class ArcadeTimedDriveMacro extends Coordinator {
    /*private final SmartTimer timer = new SmartTimer();

    private final Drive.ArcadeDrive driveForward;

    public ArcadeTimedDriveMacro ( com._604robotics.robot2019.Robot2019 robot) {
        driveForward = robot.drive.new ArcadeDrive();
    }

    protected abstract double getMovePower ();
    protected abstract double getRotatePower ();
    protected abstract double getTime ();

    @Override
    protected void begin () {
        timer.start();
    }

    @Override
    protected boolean run () {
        return timer.runUntil(getTime(), () -> {
            driveForward.movePower.set(getMovePower());
            driveForward.rotatePower.set(getRotatePower());
            driveForward.activate();
        });
    }

    @Override
    protected void end () {
        timer.stopAndReset();
    }*/
}