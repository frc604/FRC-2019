package com._604robotics.robot2019.constants;

import com._604robotics.robotnik.utils.AutonMovement;
import com._604robotics.robotnik.utils.annotations.Unreal;
import com._604robotics.robotnik.utils.annotations.Untested;
import jaci.pathfinder.Trajectory;

public class Calibration {
    private Calibration () {}

    public static final double TELEOP_DRIVE_DEADBAND = 0.3;
    public static final double TELEOP_MANIP_DEADBAND = 0.2;
    public static final double TELEOP_FACTOR = -1;

    @Unreal("Values must be updated when the real robot is designed")
    public static final double DRIVE_MOVE_PID_P = 0.0045;
    @Unreal("Values must be updated when the real robot is designed")
    public static final double DRIVE_MOVE_PID_I = 0;
    @Unreal("Values must be updated when the real robot is designed")
    public static final double DRIVE_MOVE_PID_D = 0.00;
    public static final double DRIVE_MOVE_PID_MAX = 0.85; //0.7
    public static final double DRIVE_MOVE_TOLERANCE = 100;
    public static final double SLOW_ROTATION_MODIFIER = 0.66;

    @Unreal("Values must be updated when the real robot is designed")
    public static final double DRIVE_ROTATE_PID_P = 0.003;
    public static final double DRIVE_ROTATE_PID_I = 0;
    @Unreal("Values must be updated when the real robot is designed")
    public static final double DRIVE_ROTATE_PID_D = 0.01;
    public static final double DRIVE_ROTATE_PID_MAX = 0.4;
    public static final double DRIVE_ROTATE_TOLERANCE = 80;

    public static final double DRIVE_PID_AFTER_TIMING = 0.75;
    public static final double DRIVE_PID_SAMPLE_RATE = 0.01;

    public static final double DRIVE_MOTOR_RAMP = 4;

    /* Arm Calibration */
    @Unreal("Values must be updated when robot is designed")
    public static final class Arm {
        public static final double CLICKS_FULL_ROTATION = 10240; // One rotation of the encoder in clicks

        public static final double LOW_SETPOINT = 4116; // Ready to intake from front
        public static final double STOW_SETPOINT = 0 - 3; // Starting position
        public static final double OUTPUT_SETPOINT = 1182; // Output from front TODO: be higher
        public static final double ROCKET_SETPOINT = 2606;
        public static final double VERTICAL_POSITION = 440 ; // The arm is straight up
        public static final double HORIZONTAL_POSITION = -1527; // The arm is horizontally backwards NOTE: Did not modify with HARDSTOP b.c. it works still
        public static final double BACK_ROCKET_SETPOINT = -985;
        public static final double BACK_CARGO_SETPOINT = -377;

        public static final double HARDSTOP_CLOSE_POSITION = -25;
        //HORIZONTAL_POSITION = Arm balance point - (CLICKS_FULL_ROTATION / 4)

        public static final double MIN_ENCODER_VAL = 0; // The lowest value the encoder can have
        public static final double MAX_ENCODER_VAL = 0; // The largest value the encoder can have

        public static final double kP = -0.0004;
        public static final double kI = 0;
        public static final double kD = -0.00025;
        public static final double kF = -0.075;
		//-0.05

        public static final double SCALE_JOYSTICK = 0.6; // Multiply the input joystick by this
    }

    public static final double HATCH_PUSH_TIME = 0.5;
    public static final double HATCH_BACK_TIME = 0.5;
    public static final double HATCH_PULL_TIME = 0.5;
    public static final double HATCH_RELEASE_TIME = 0.5;

    @Unreal("Values must be updated when the real robot is designed")
    public static final AutonMovement.DriveTrainProperties DRIVE_PROPERTIES
    = new AutonMovement.DriveTrainProperties(4096, 3.81 * 2, 0.0508, 3.0, 2* Math.PI);
    // second to last = coefficient, second value = offset

    /* Marionette */
    public static final boolean AUTO_APPEND_TIMESTAMP = true;

    public static final String CUSTOM_PRIMARY = "single00.switchLeft.marionette";
    public static final String CUSTOM_SECONDARY = "half00.switchRight.marionette";

    /* Pathfinder */
    @Unreal("Values must be updated when the real robot is designed")
    public final class Pathfinder {
        public static final double ROBOT_WIDTH = 0.66;
        public static final double KP = 2;
        public static final double KI = 0;
        public static final double KD = 0.0;
        public static final double KV = 1.0/2.3;
        public static final double KA = 0.03;
        public static final double K_KAPPA = 0.09;
        public static final double K_PTHETA_0 = 2.8;
        public static final double K_PTHETA_DECAY = 0.7;
        public static final double K_DTHETA_0 = 0.02;
        public static final double K_DTHETA_DECAY = 0.3;
    }

    public static final Trajectory.Config PATHFINDER_CONFIG = new Trajectory.Config(
            Trajectory.FitMethod.HERMITE_QUINTIC,
            Trajectory.Config.SAMPLES_LOW,
            0.025, 1.6, 1.1, 3.5 );

    @Untested("Currently matches value in PixyMon")
    public static final int PIXY_MAX_BLOCKS = 20; // May or may not have to match PixyMon settings...

    // :5800 includes framerate and crosshair :5802 does not :5801 is the control interface
    public static final String LIMELIGHT_URL = "http://10.6.4.101:5802";
    public static final int LIMELIGHT_FPS = 90; // Default: Uncapped (Normally around 90)
    public static final int LIMELIGHT_RES_X = 320; // Default: 320
    public static final int LIMELIGHT_RES_Y = 240; // Default: 240
    @Unreal("Needs to be measured")
    public static final double LIMELIGHT_HEIGHT = 24.5; // Height of limelight from the ground in inches
    @Unreal("Needs to be measured")
    public static final double LIMELIGHT_ANGLE = 80; // Angle of the limelight relative to the plane of the robot in degrees
                                                    // If negative, pointed down, positive is pointed up
    @Unreal("Needs to be measured")
    public static final double TARGET_HEIGHT = 28; // Height of the center of the vision target in inches

    public static final double LIMELIGHT_ANGLE_TOLERANCE = 0.005;
    public static final double LIMELIGHT_DIST_TOLERANCE = 1;
    public static final int LIMELIGHT_DRIVER_PIPE = 9; // The pipeline to use for driver usage, NOT tracking
    public static final int LIMELIGHT_VISION_PIPE = 0; // Pipeline for vision tracking by default
}
