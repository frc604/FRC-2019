package com._604robotics.robot2019.constants;

import com._604robotics.robotnik.utils.annotations.Unreal;
import com._604robotics.robotnik.utils.annotations.Untested;

public class Calibration {

  public static final double TELEOP_DRIVE_DEADBAND = 0.3;
  public static final double TELEOP_MANIP_DEADBAND = 0.2;
  public static final double TELEOP_FACTOR = -1;

  @Unreal("Values must be updated when the real robot is designed")
  public static final double DRIVE_MOVE_PID_P = 0.0045;

  @Unreal("Values must be updated when the real robot is designed")
  public static final double DRIVE_MOVE_PID_I = 0;

  @Unreal("Values must be updated when the real robot is designed")
  public static final double DRIVE_MOVE_PID_D = 0.00;

  public static final double DRIVE_MOVE_PID_MAX = 0.85; // 0.7
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
  public static final class Flywheel {
    public static final double kP = 0.055;
    public static final double kD = 0;
  }

  /* Marionette */
  public static final boolean AUTO_APPEND_TIMESTAMP = true;

  public static final String CUSTOM_PRIMARY = "single00.switchLeft.marionette";
  public static final String CUSTOM_SECONDARY = "half00.switchRight.marionette";

  @Untested("Currently matches value in PixyMon")
  public static final int PIXY_MAX_BLOCKS = 20; // May or may not have to match PixyMon settings...

  // :5800 includes framerate and crosshair :5802 does not :5801 is the control interface
  public static final String LIMELIGHT_URL = "http://10.6.4.101:5802";
  public static final int LIMELIGHT_FPS = 90; // Default: Uncapped (Normally around 90)
  public static final int LIMELIGHT_RES_X = 320; // Default: 320
  public static final int LIMELIGHT_RES_Y = 240; // Default: 240

  @Unreal("Needs to be measured")
  public static final double LIMELIGHT_HEIGHT =
      24.5; // Height of limelight from the ground in inches

  @Unreal("Needs to be measured")
  public static final double LIMELIGHT_ANGLE =
      80; // Angle of the limelight relative to the plane of the robot in degrees
  // If negative, pointed down, positive is pointed up
  @Unreal("Needs to be measured")
  public static final double TARGET_HEIGHT =
      28; // Height of the center of the vision target in inches

  public static final double LIMELIGHT_ANGLE_TOLERANCE = 0.005;
  public static final double LIMELIGHT_DIST_TOLERANCE = 1;
  public static final int LIMELIGHT_DRIVER_PIPE =
      9; // The pipeline to use for driver usage, NOT tracking
  public static final int LIMELIGHT_VISION_PIPE = 0; // Pipeline for vision tracking by default
}
