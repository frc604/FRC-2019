package com._604robotics.robotnik.prefabs.devices;

public abstract class AbsoluteEncoder implements Encoder {

  /** Sets the zero of the encoder to its current value. */
  public abstract void setZero();

  /**
   * Sets the zero of an encoder.
   *
   * @param zero Zero value to set.
   */
  public abstract void setZero(double zero);

  /**
   * Sets the zero of the encoder to an angle.
   *
   * @param zeroAngle Zero angle to set.
   */
  public abstract void setZeroAngle(double zeroAngle);

  /**
   * Gets the (raw, non-zeroed) voltage value of the encoder.
   *
   * @return The (raw, non-zeroed) voltage value of the encoder.
   */
  public abstract double getRawVoltage();

  /**
   * Gets the (zeroed) voltage value of the encoder.
   *
   * @return The (zeroed) voltage value of the encoder.
   */
  public abstract double getVoltage();

  /**
   * Gets the value of the encoder, translating angle to clicks where 250 clicks equals 360 degrees
   *
   * @return the value of {@link AbsoluteEncoder#getAngle()} * 250/360
   */
  @Override
  public double getValue() {
    return getAngle() * 250 / 360;
  }

  /**
   * Gets the (zeroed) angle of the encoder.
   *
   * @return The (zeroed) angle of the encoder.
   */
  public abstract double getAngle();

  /**
   * Gets the (raw, non-zeroed) angle value of the encoder.
   *
   * @return The (raw, non-zeroed) angle of the encoder.
   */
  public abstract double getRawAngle();
}
