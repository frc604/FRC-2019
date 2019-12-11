package com._604robotics.robotnik.prefabs.controller;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Subclass of ProfiledPIDController that has a feedforward for rotating arms. This subclass
 * requires an PIDSource as the PIDSource and uses only continuous error. Zero is assumed to be
 * horizontal. Users are responsible for properly zeroing the PIDSource beforehand.
 */
public class ProfiledRotatingArmPIDController extends NewExtendableProfiledPIDController {
  private double encoderPeriod = 360;
  private double zeroOffset = 0;
  private double m_kf = 0.0;
  private double m_maxOutput = 1.0;
  private double m_minOutput = -1.0;

  public ProfiledRotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      double Kf,
      TrapezoidProfile.Constraints constraints,
      double period,
      DoubleSupplier source,
      DoubleConsumer output) {
    super(Kp, Ki, Kd, period, constraints, source, output);
    this.m_kf = Kf;
  }

  public ProfiledRotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      double Kf,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier source,
      DoubleConsumer output) {
    super(Kp, Ki, Kd, 0.02, constraints, source, output);
    this.m_kf = Kf;
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

  public void setOutputRange(double min, double max) {
    this.m_minOutput = min;
    this.m_maxOutput = max;
  }

  /**
   * Overridden feed forward part of PIDController. This is a physically based model which multiplies
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
