package com._604robotics.robot2019.constants;

import com._604robotics.robotnik.utils.annotations.Unreal;

@Unreal("Assign all ports when they are decided in design")
public class Ports {
    private Ports () {}
    
    /* Solenoids */
    public static final int SHIFTER_A = 0;
    public static final int SHIFTER_B = 2;
    
    /* Victor Motors */
    public static final int DRIVE_FRONT_LEFT_MOTOR = 1;
    public static final int DRIVE_REAR_LEFT_MOTOR = 2;
    public static final int DRIVE_FRONT_RIGHT_MOTOR = 3;
    public static final int DRIVE_REAR_RIGHT_MOTOR = 4;

    /* Arm Motors */
    public static final int ARM_LEFT_MOTOR = 5;
    public static final int ARM_RIGHT_MOTOR = 6;

    /* Intake Motor */
    public static final int INTAKE_MOTOR = 2;

    /* Hatch Control */
    public static final int HOOK_A = 0;
    public static final int HOOK_B = 0;
    public static final int HATCH_LEFT_SLIDE_A= 0;
    public static final int HATCH_LEFT_SLIDE_B = 0;
    public static final int HATCH_RIGHT_SLIDE_A = 0;
    public static final int HATCH_RIGHT_SLIDE_B = 0;
    public static final int HATCH_LEFT_SWITCH = 0;
    public static final int HATCH_RIGHT_SWITCH = 0;
    public static final int HATCH_PUSHER_A = 0;
    public static final int HATCH_PUSHER_B = 0;

    /* Tilter Control */
    public static final int TILT_MOTOR = 0;
    public static final int TILT_SWITCH = 0;

    /* Digital Inputs */
    public static final int ENCODER_LEFT_A = 1;
    public static final int ENCODER_LEFT_B = 0;
    public static final int ENCODER_RIGHT_A = 2;
    public static final int ENCODER_RIGHT_B = 3;
    
    /* CAN Motors */
    public static final int COMPRESSOR = 0;
    public static final int PDP_MODULE = 1;
    
}
