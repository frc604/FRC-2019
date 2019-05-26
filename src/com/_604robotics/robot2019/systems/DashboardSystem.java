package com._604robotics.robot2019.systems;

import com._604robotics.robotnik.Coordinator;

public class DashboardSystem extends Coordinator {
    private final com._604robotics.robot2019.Robot2019 robot;

    public DashboardSystem ( com._604robotics.robot2019.Robot2019 robot) {
        this.robot = robot;
    }

    @Override
    public boolean run () {
        

        //robot.dashboard.totalCurrent.set(robot.powermonitor.totalPortCurrent.get());
        
        //robot.powermonitor.initDashboardSendables();
        /* Limelight Testing */
        /*
        robot.dashboard.limelightTarget.set(robot.limelight.limelightHasTargets.get());
        robot.dashboard.limelightX.set(robot.limelight.limelightX.get());
        robot.dashboard.limelightY.set(robot.limelight.limelightY.get());
        robot.dashboard.limelightArea.set(robot.limelight.limelightArea.get());
        robot.dashboard.limelightSkew.set(robot.limelight.limelightSkew.get());
        robot.dashboard.limelightDistance.set(robot.limelight.limelightDistance.get());

        //robot.limelight.limelightLED.set(robot.dashboard.limelightLEDState.get().ordinal());
        robot.limelight.limelightStreamMode.set(robot.dashboard.limelightStreamMode.get().ordinal());
        robot.limelight.limelightSnapshotEnabled.set(robot.dashboard.limelightSnapshot.get());

        if( robot.limelight.scan.isRunning() ) {
            robot.limelight.limelightPipeline.set(
                Math.min(0, Math.max(9, robot.dashboard.limelightPipeline.get().intValue())) );
        }
        */
        
        switch( robot.dashboard.marionetteRecorder.get() ) {
            case MANUAL:
                robot.dashboard.writeFile.set(robot.dashboard.filePrefix.get() + robot.dashboard.marionetteFile.get());
                break;
            default:
                break;
        }
        
        switch( robot.dashboard.marionetteOutput.get() ) {
            case MANUAL:
                robot.dashboard.primaryReadFile.set(robot.dashboard.filePrefix.get() + robot.dashboard.marionetteFile.get());
                break;
            default:
                break;
        }

        // Leave at end of function
        return true;
    }
}
