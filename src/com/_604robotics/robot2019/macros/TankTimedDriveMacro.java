package com._604robotics.robot2019.macros;

import com._604robotics.robot2019.Robot2017;
import com._604robotics.robot2019.modules.Drive;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;

public abstract class TankTimedDriveMacro extends Coordinator {
    private final SmartTimer timer = new SmartTimer();

    private final Drive.TankDrive driveForward;

    public TankTimedDriveMacro (Robot2017 robot) {
        driveForward = robot.drive.new TankDrive();
    }

    protected abstract double getLeftPower ();
    protected abstract double getRightPower ();
    protected abstract double getTime ();

    @Override
    protected void begin () {
        timer.start();
    }

    @Override
    protected boolean run () {
        return timer.runUntil(getTime(), () -> {
            driveForward.leftPower.set(getLeftPower());
            driveForward.rightPower.set(getRightPower());
            driveForward.activate();
        });
    }

    @Override
    protected void end () {
        timer.stopAndReset();
    }
}