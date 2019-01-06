package com._604robotics.robot2019.systems;

import com._604robotics.robot2019.Robot2017;
import com._604robotics.robotnik.Coordinator;

public class PickupSystem extends Coordinator {
    private final Robot2017 robot;

    public PickupSystem (Robot2017 robot) {
        this.robot = robot;
    }

    @Override
    public boolean run () {
        if (robot.flipFlop.retract.isRunning() && !robot.flipFlop.retract.completed.get()) {
            robot.intake.suck.activate();
        }
        return true;
    }
}