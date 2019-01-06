package com._604robotics.robot2019.modules;

import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.modules.DashboardModule;

public class Dashboard extends DashboardModule {
    public final Input<Integer> leftDriveClicks = addDashboardInput("leftDriveClicks", 0);
    public final Input<Integer> rightDriveClicks = addDashboardInput("rightDriveClicks", 0);

    public final Input<Double> leftDriveRate = addDashboardInput("leftDriveRate", 0.0);
    public final Input<Double> rightDriveRate = addDashboardInput("rightDriveRate", 0.0);
    
    public final Input<Double> xAccel = addDashboardInput("X Acceleration (g)", 0.0);
    public final Input<Double> yAccel = addDashboardInput("Y Acceleration (g)", 0.0);
    public final Input<Double> zAccel = addDashboardInput("Z Acceleration (g)", 0.0);

    public final Input<Double> totalCurrent = addDashboardInput("Current Drawn",0.0);

    public final Input<Boolean> XboxFlipped = addDashboardInput("XboxFlipped", false);
    
    public enum AutonMode {
        OFF,
        // Following are actual strategy selections

        // Calibration autons to verify angles and distances
        ROTATE_LEFT_TEST,
        ROTATE_RIGHT_TEST,
        FAILSAFE_FORWARD_12,
        FAILSAFE_BACKWARD_12,

        // Demo auton into which arbitrary stuff can be stashed for testing
        DEMO_NEW_AUTON,
        MARIONETTE
    }
    
    public enum DriveMode {
        OFF,
        ARCADE,
        TANK,
        DYNAMIC
    }
    
    public enum MarionetteRecorder {
    	MANUAL,
    }
    
    public enum MarionetteOutput {
    	MANUAL,
    }

    public final Output<AutonMode> autonMode = addDashboardOutput("autonMode", AutonMode.DEMO_NEW_AUTON, AutonMode.class);
    
    public final Output<DriveMode> driveMode = addDashboardOutput("driveMode", DriveMode.DYNAMIC, DriveMode.class);

    public final Output<MarionetteRecorder> marionetteRecorder = addDashboardOutput("marionetteRecorder", MarionetteRecorder.MANUAL, MarionetteRecorder.class);
    
    public final Output<MarionetteOutput> marionetteOutput = addDashboardOutput("marionetteOutput", MarionetteOutput.MANUAL, MarionetteOutput.class);
    
    public final Output<Boolean> recordAuton = addDashboardOutput("recordAuton", false);
    public final Output<String> marionetteFile = addDashboardOutput("marionetteFile", "autonomous.marionette");
    public final Output<String> filePrefix = addDashboardOutput("filePrefix", "");
    
    public final Input<String> primaryReadFile = addDashboardInput("Primary Read File: ", "");
    public final Input<String> secondaryReadFile = addDashboardInput("Secondary Read File: ", "");
    public final Input<String> writeFile = addDashboardInput("Write File: ", "");
    
    public final Input<String> manualPrimaryReadFile = addDashboardInput("Manual Primary Read File: ", "");
    public final Input<String> manualSecondaryReadFile = addDashboardInput("Manual Secondary Read File: ", "");
    
    public Dashboard () {
        super(Dashboard.class);
    }
}
