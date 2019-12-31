package com._604robotics.robotnik.prefabs.inputcontroller;

import edu.wpi.first.wpilibj.Joystick;

/** An axis on a controller. */
public class ControllerAxis {
  private final Joystick joystick;
  private final int axis;

  private double deadband = 0D;
  private double factor = 1D;

  /**
   * Creates a controller axis.
   *
   * @param joystick Joystick containing the axis.
   * @param axis Axis to represent.
   */
  public ControllerAxis(Joystick joystick, int axis) {
    this.joystick = joystick;
    this.axis = axis;
  }

  public double get() {
    final double value = this.joystick.getRawAxis(this.axis) * this.factor;

    return Math.abs(value) < this.deadband ? 0D : value;
  }

  /**
   * Sets the deadband of the axis.
   *
   * @param deadband Deadband to set.
   */
  public void setDeadband(double deadband) {
    this.deadband = deadband;
  }

  /**
   * Sets the multiplication factor of the axis.
   *
   * @param factor Factor to set.
   */
  public void setFactor(double factor) {
    this.factor = factor;
  }

  /** Inverts the multiplication factor of the axis. */
  public void invertFactor(boolean inverted) {
    // True is positive, False is negative
    boolean factorSign = Math.signum(this.factor) == 1;

    // If positive and should be negative
    if (factorSign && inverted) {
      this.factor *= -1;

      // If negative and shouldn't be negative
    } else if (!factorSign && !inverted) {
      this.factor = Math.abs(this.factor);
    }

    // Anything that falls through means the current sign is the same as desired
  }
}
