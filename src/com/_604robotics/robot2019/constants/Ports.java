package com._604robotics.robot2019.constants;

import com._604robotics.robotnik.utils.annotations.Unreal;

@Unreal("Assign all ports when they are decided in design")
public class Ports {
    private Ports () {}
    
    /* Victor Motors */
    public static final int DRIVE_FRONT_LEFT_MOTOR = 1;
    public static final int DRIVE_MIDDLE_LEFT_MOTOR = 2;
    public static final int DRIVE_REAR_LEFT_MOTOR = 3;
    public static final int DRIVE_FRONT_RIGHT_MOTOR = 4;
    public static final int DRIVE_MIDDLE_RIGHT_MOTOR = 5;
    public static final int DRIVE_REAR_RIGHT_MOTOR = 6;

    /* Arm Motors */
    public static final int ARM_LEFT_MOTOR = 2;
    public static final int ARM_RIGHT_MOTOR = 3;

    /* Intake Motor */
    public static final int INTAKE_MOTOR = 11;
    public static final int INTAKE_LIMIT = 4;

    /* Hatch Control */
    public static final int HOOK_A = 0;
    public static final int HOOK_B = 4;
    public static final int HATCH_LEFT_SLIDE_A= 1;
    public static final int HATCH_LEFT_SLIDE_B = 5;

    public static final int HATCH_LEFT_SWITCH = 0;
    public static final int HATCH_RIGHT_SWITCH = 0;
    public static final int HATCH_PUSHER_A = 2;
    public static final int HATCH_PUSHER_B = 6;

    /* Hardstop */
    public static final int HARDSTOP_A = 2;
    public static final int HARDSTOP_B = 3;

    /* Tilter Control */
    public static final int TILT_MOTOR = 1;
    public static final int TILT_SWITCH = 9;

    /* Digital Inputs */
    public static final int ENCODER_LEFT_A = 1;
    public static final int ENCODER_LEFT_B = 0;
    public static final int ENCODER_RIGHT_A = 3;
    public static final int ENCODER_RIGHT_B = 2;
    public static final int HORIZGYRO = 0;
    
    /* CAN Motors */
    public static final int COMPRESSOR = 0; // AKA PCM
    public static final int PDP_MODULE = 1;
    
}
