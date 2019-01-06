package com._604robotics.robot2019.systems;

import com._604robotics.robot2019.Robot2017;
import com._604robotics.robotnik.Coordinator;

public class DashboardSystem extends Coordinator {
    private final Robot2017 robot;

    public DashboardSystem (Robot2017 robot) {
        this.robot = robot;
    }

    @Override
    public boolean run () {
        robot.intake.power.set(robot.dashboard.intakePower.get());
        robot.flipFlop.transitionTime.set(robot.dashboard.flipFlopTransitionTime.get());
        robot.dashboard.leftDriveClicks.set(robot.drive.leftClicks.get());
        robot.dashboard.rightDriveClicks.set(robot.drive.rightClicks.get());
        robot.dashboard.leftDriveRate.set(robot.drive.leftClickRate.get());
        robot.dashboard.rightDriveRate.set(robot.drive.rightClickRate.get());
        //robot.dashboard.gyroAngle.set(robot.drive.gyroAngle.get());
        robot.dashboard.totalCurrent.set(robot.powerMonitor.totalPortCurrent.get());
        //robot.dashboard.topRate.set(robot.shooter.topRate.get());
        //robot.dashboard.isCharged.set(robot.shooter.isCharged.get());
        robot.dashboard.gearDetected.set(robot.intake.gearDetected.get());
        
        return true;
    }
}
