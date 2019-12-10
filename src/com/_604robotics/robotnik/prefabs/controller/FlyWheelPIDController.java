package com._604robotics.robotnik.prefabs.controller;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Subclass of PIDController that has a feedforward for spinning fly wheels. This subclass requires
 * a double supplier as the source and a double consumer as the output and uses only continuous
 * error. Typically, a flywheel controller only needs a proportional term, but an overload is
 * available if a derivative term is needed. The controller also handles feedforward calculation
 * internally, and requires a {@link edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward} with
 * Ks, and Kv coefficients.
 */
public class FlyWheelPIDController extends NewExtendablePIDController {
  private final SimpleMotorFeedforward m_feedforward;

  /**
   * Allocates a FlyWheelPIDController with a given Kp and feedforward. The controller is run at a
   * default period of 0.02s.
   *
   * @param Kp The P term of the controller.
   * @param feedforward A SimpleMotorFeedForward with Ks, Kv, and Ka values.
   * @param source A lambda supplying the controller measurement.
   * @param output A lambda consuming the controller output.
   */
  public FlyWheelPIDController(
      double Kp, SimpleMotorFeedforward feedforward, DoubleSupplier source, DoubleConsumer output) {
    super(Kp, 0, 0, source, output);
    this.m_feedforward = feedforward;
  }

  /**
   * Allocates a FlyWheelPIDController with a given Kp, Kd and feedforward. The controller is run at
   * a default period of 0.02s.
   *
   * @param Kp The P term of the controller.
   * @param Kd The D term of the controller.
   * @param feedforward A SimpleMotorFeedForward with Ks, Kv, and Ka values.
   * @param source A lambda supplying the controller measurement.
   * @param output A lambda consuming the controller output.
   */
  public FlyWheelPIDController(
      double Kp,
      double Kd,
      SimpleMotorFeedforward feedforward,
      DoubleSupplier source,
      DoubleConsumer output) {
    super(Kp, 0, Kd, source, output);
    this.m_feedforward = feedforward;
  }

  /**
   * Allocates a FlyWheelPIDController with a given Kp and feedforward. The controller is run at a
   * specified period.
   *
   * @param Kp The P term of the controller.
   * @param feedforward A SimpleMotorFeedForward with Ks, Kv, and Ka values.
   * @param period The period of the controller.
   * @param source A lambda supplying the controller measurement.
   * @param output A lambda consuming the controller output.
   */
  public FlyWheelPIDController(
      double Kp,
      SimpleMotorFeedforward feedforward,
      double period,
      DoubleSupplier source,
      DoubleConsumer output) {
    super(Kp, 0, 0, period, source, output);
    this.m_feedforward = feedforward;
  }

  /**
   * Allocates a FlyWheelPIDController with a given Kp, Kd and feedforward. The controller is run at
   * a specified period.
   *
   * @param Kp The P term of the controller.
   * @param Kd The D term of the controller.
   * @param feedforward A SimpleMotorFeedForward with Ks, Kv, and Ka values.
   * @param period The period of the controller.
   * @param source A lambda supplying the controller measurement.
   * @param output A lambda consuming the controller output.
   */
  public FlyWheelPIDController(
      double Kp,
      double Kd,
      SimpleMotorFeedforward feedforward,
      double period,
      DoubleSupplier source,
      DoubleConsumer output) {
    super(Kp, 0, Kd, period, source, output);
    this.m_feedforward = feedforward;
  }

  /**
   * Overridden feed forward part of PIDController. This is a physically based model which
   * calculates the feed forward term through from a {@link
   * edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward}. The feed forward accounts for static
   * friction and friction during movement though the Ks, Kv, and Ka coefficients.
   *
   * @return the feed forward value
   */
  @Override
  protected double calculateFeedForward() {
    double velSetpoint;
    double prevVel;
    double curVel;
    double period;

    m_thisMutex.lock();
    try {
      /**
       * We need to get the velocity setpoint, and the current and previous velocity measurements
       * along with the controller period to calculate the current acceleration.
       */
      velSetpoint = getSetpoint();
      prevVel = getMeasurement();
      curVel = m_pidSource.getAsDouble();
      period = getPeriod();
    } finally {
      m_thisMutex.unlock();
    }

    // Acceleration = delta V / delta T
    var accelInput = (curVel - prevVel) / period;

    // Calculate the necessary feedforward term for for the given velocity state and acceleration
    // input.
    return m_feedforward.calculate(velSetpoint, accelInput)
        / 12; // Feedforward is in terms of volts
  }
}
