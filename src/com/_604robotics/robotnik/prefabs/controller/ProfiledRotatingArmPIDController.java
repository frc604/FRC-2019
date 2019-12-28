package com._604robotics.robotnik.prefabs.controller;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Subclass of PIDController that has a feedforward for rotating arms. This subclass requires an
 * PIDSource as the PIDSource and uses only continuous error. Zero is assumed to be horizontal.
 * Users are responsible for properly zeroing the PIDSource beforehand.
 */
public class ProfiledRotatingArmPIDController extends ProfiledPIDController {
  private ArmFeedforward m_feedforward;
  private double m_encoderPeriod = 360;
  private double m_zeroOffset = 0;

  public ProfiledRotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier source,
      DoubleConsumer output) {
    super(Kp, Ki, Kd, constraints, source, output);
  }

  public ProfiledRotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier source,
      DoubleConsumer output,
      double period) {
    super(Kp, Ki, Kd, constraints, period, source, output);
  }

  public ProfiledRotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      ArmFeedforward feedforward,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier source,
      DoubleConsumer output) {
    super(Kp, Ki, Kd, constraints, source, output);
    this.m_feedforward = feedforward;
  }

  public ProfiledRotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      ArmFeedforward feedforward,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier source,
      DoubleConsumer output,
      double period) {
    super(Kp, Ki, Kd, constraints, period, source, output);
    this.m_feedforward = feedforward;
  }

  public double getEncoderPeriod() {
    return m_encoderPeriod;
  }

  public void setEncoderPeriod(double encoderPeriod) {
    this.m_encoderPeriod = encoderPeriod;
  }

  public void setFeedforwardZeroOffset(double zeroOffset) {
    this.m_zeroOffset = zeroOffset;
  }

  /**
   * Overridden feed forward part of ProfiledPIDController. This is a physically based model which
   * multiplies feed forward coefficient by cosine. The feedforward calculates the expected torque
   * needed to hold an arm steady, scaled to motor power.
   *
   * @return the feed forward value
   */
  @Override
  protected double calculateFeedForward() {
    // Calculate cosine for torque factor
    double angle;
    m_thisMutex.lock();
    try {
      angle = m_pidSource.getAsDouble() - m_zeroOffset;
    } finally {
      m_thisMutex.unlock();
    }

    // Cosine is periodic so sawtooth wraparound is not a concern
    angle /= m_encoderPeriod;
    angle *= 360;

    /*
     Dividing by 12 to convert volts to motor output power. We can ignore the current voltage of the battery as voltage saturation is available on talons.
    */
    return m_feedforward.calculate(angle, 0) / 12;
  }
}
