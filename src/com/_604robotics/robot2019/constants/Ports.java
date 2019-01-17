package com._604robotics.robot2019.constants;

import com._604robotics.robotnik.utils.annotations.Unreal;

@Unreal("Assign all ports when they are decided in design")
public class Ports {
    private Ports () {}
    
    /* Solenoids */
    public static final int SHIFTER_A = 0;
    public static final int SHIFTER_B = 2;
    
    public static final int CLAMP_A = 1;
    public static final int CLAMP_B = 3;
    
    /* Victor Motors */
    // Drive
    public static final int DRIVE_FRONT_LEFT_MOTOR = 2;
    public static final int DRIVE_CENTER_LEFT_MOTOR = 4;
    public static final int DRIVE_REAR_LEFT_MOTOR = 3;
    public static final int DRIVE_FRONT_RIGHT_MOTOR = 0;
    public static final int DRIVE_CENTER_RIGHT_MOTOR = 5;
    public static final int DRIVE_REAR_RIGHT_MOTOR = 1;

    /* Digital Inputs */
    public static final int ENCODER_LEFT_A = 0;
    public static final int ENCODER_LEFT_B = 1;
    public static final int ENCODER_RIGHT_A = 2;
    public static final int ENCODER_RIGHT_B = 3;
    
    /* CAN Motors */
    public static final int COMPRESSOR = 0;
    public static final int PDP_MODULE = 1;
    
}
