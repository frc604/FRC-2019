package com._604robotics.robotnik.prefabs.controller;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Subclass of PIDController that has a feedforward for rotating arms. This subclass requires an
 * PIDSource as the PIDSource and uses only continuous error. Zero is assumed to be horizontal.
 * Users are responsible for properly zeroing the PIDSource beforehand.
 */
public class RotatingArmPIDController extends NewExtendablePIDController {
  private double encoderPeriod = 360;
  private double zeroOffset = 0;

  public RotatingArmPIDController(
      double Kp, double Ki, double Kd, DoubleSupplier source, DoubleConsumer output) {
    super(Kp, Ki, Kd, source, output);
  }

  public RotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      DoubleSupplier source,
      DoubleConsumer output,
      double period) {
    super(Kp, Ki, Kd, source, output, period);
  }

  public RotatingArmPIDController(
      double Kp, double Ki, double Kd, double Kf, DoubleSupplier source, DoubleConsumer output) {
    super(Kp, Ki, Kd, Kf, source, output);
  }

  public RotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      double Kf,
      DoubleSupplier source,
      DoubleConsumer output,
      double period) {
    super(Kp, Ki, Kd, Kf, period, source, output);
  }

  public double getEncoderPeriod() {
    return encoderPeriod;
  }

  public void setEncoderPeriod(double encoderPeriod) {
    this.encoderPeriod = encoderPeriod;
  }

  public void setFeedforwardZeroOffset(double zeroOffset) {
    this.zeroOffset = zeroOffset;
  }

  /**
   * Overriden feed forward part of PIDController. This is a physically based model which multiplies
   * feed forward coefficient by cosine. The feedforward calculates the expected torque needed to
   * hold an arm steady, scaled to motor power.
   *
   * @return the feed forward value
   */
  @Override
  protected double calculateFeedForward() {
    // Calculate cosine for torque factor
    double angle;
    double fValue;
    m_thisMutex.lock();
    try {
      angle = m_pidSource.getAsDouble() - zeroOffset;
      fValue = getF();
    } finally {
      m_thisMutex.unlock();
    }
    // Cosine is periodic so sawtooth wraparound is not a concern
    angle /= encoderPeriod;
    angle *= (2 * Math.PI);
    double cosine = Math.cos(angle);
    return fValue * cosine;
  }
}
