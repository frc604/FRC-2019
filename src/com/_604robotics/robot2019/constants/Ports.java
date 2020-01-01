package com._604robotics.robot2019.constants;

import com._604robotics.robotnik.utils.annotations.Unreal;

@Unreal("Assign all ports when they are decided in design")
public class Ports {
  private Ports() {}

  /* Flywheel Motors */
  public static final int FLYWHEEL_MOTOR = 0;
  public static final int TRANSFER_MOTOR = 1;
  public static final int FEEDER_MOTOR = 2;

  /* CAN Devices */
  public static final int COMPRESSOR = 0; // AKA PCM
  public static final int PDP_MODULE = 1;
}
