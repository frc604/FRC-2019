/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* This is 604's extendable version which has been                            */
/* re-written for the 2020 season                                             */
/*----------------------------------------------------------------------------*/

package com._604robotics.robotnik.prefabs.controller;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpiutil.math.MathUtils;

import java.util.TimerTask;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;


/**
 * Implements a PID control loop.
 */
public class ExtendablePIDController implements Sendable, AutoCloseable {
  private static int instances;

  // Factor for "proportional" control
  private double m_Kp;

  // Factor for "integral" control
  private double m_Ki;

  // Factor for "derivative" control
  private double m_Kd;

  // Factor for "feedforward" term
  private double m_Kf;

  // The period (in seconds) of the loop that calls the controller
  private final double m_period;

  private double m_maximumIntegral = 1.0;

  private double m_minimumIntegral = -1.0;

  // Maximum input - limit setpoint to this
  private double m_maximumInput;

  // Minimum input - limit setpoint to this
  private double m_minimumInput;

  // Input range - difference between maximum and minimum
  private double m_inputRange;

  // Do the endpoints wrap around? eg. Absolute encoder
  private boolean m_continuous;

  // Is the pid controller enabled
  private boolean m_enabled = false;

  // The error at the time of the most recent call to calculate()
  private double m_positionError;
  private double m_velocityError;

  // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
  private double m_prevError;

  // The sum of the errors for use in the integral calc
  private double m_totalError;

  // The percentage or absolute error that is considered at setpoint.
  private double m_positionTolerance = 0.05;
  private double m_velocityTolerance = Double.POSITIVE_INFINITY;

  private double m_setpoint;
  private double m_result = 0.0;

  private DoubleConsumer m_pidOutput;
  private DoubleSupplier m_pidSource;

  private DoubleSupplier m_origSource;
  

  ReentrantLock m_thisMutex = new ReentrantLock();
  ReentrantLock m_pidOutputMutex = new ReentrantLock();

  java.util.Timer m_controlLoop;
  Timer m_setpointTimer;
  

  private class PIDTask extends TimerTask {
    private ExtendablePIDController m_controller;

    PIDTask(ExtendablePIDController controller) {
      requireNonNullParam(controller, "controller", "Given PIDController was null");

      m_controller = controller;
    }

    @Override
    public void run() {
      m_controller.calculate();
    }
  }

  public ExtendablePIDController(double Kp, double Ki, double Kd, DoubleSupplier pidSource,
  DoubleConsumer pidOutput) {
    this(Kp, Ki, Kd, 0.0, 0.02, pidSource, pidOutput);
  }

  public ExtendablePIDController(double Kp, double Ki, double Kd, DoubleSupplier pidSource,
  DoubleConsumer pidOutput, double period) {
    this(Kp, Ki, Kd, 0.0, period, pidSource, pidOutput);
  }

  /**
   * Allocates a ExtendablePIDController with the given constants for Kp, Ki, and Kd and a default period of
   * 0.02 seconds.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param Kf The feedforward term.
   */
  public ExtendablePIDController(double Kp, double Ki, double Kd, double Kf, DoubleSupplier pidSource,
  DoubleConsumer pidOutput) {
    this(Kp, Ki, Kd, Kf, 0.02, pidSource, pidOutput);
  }

  /**
   * Allocates a ExtendablePIDController with the given constants for Kp, Ki, and Kd.
   *
   * @param Kp     The proportional coefficient.
   * @param Ki     The integral coefficient.
   * @param Kd     The derivative coefficient.
   * @param period The period between controller updates in seconds.
   */
  public ExtendablePIDController(double Kp, double Ki, double Kd, double Kf, double period, DoubleSupplier source,
  DoubleConsumer output) {
    requireNonNullParam(source, "pidSource", "Null PIDSource was given");
    requireNonNullParam(output, "pidOutput", "Null PIDOutput was given");

    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;

    m_period = period;

    m_pidSource = source;
    m_pidOutput = output;

    m_origSource = source;

    m_controlLoop = new java.util.Timer();
    m_setpointTimer = new Timer();
    m_setpointTimer.start();

    m_controlLoop.schedule(new PIDTask(this), 0L, (long) (m_period * 1000));

    instances++;
    SendableRegistry.addLW(this, "PIDController", instances);

    HAL.report(tResourceType.kResourceType_PIDController, instances);
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
    m_controlLoop.cancel();
    m_thisMutex.lock();
    try {
      m_pidOutput = null;
      m_pidSource = null;
      m_controlLoop = null;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the PID Controller gain parameters.
   *
   * <p>Set the proportional, integral, and differential coefficients.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   */
  @SuppressWarnings("ParameterName")
  public void setPID(double Kp, double Ki, double Kd) {
    m_thisMutex.lock();
    try {
      m_Kp = Kp;
      m_Ki = Ki;
      m_Kd = Kd;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the Proportional coefficient of the PID controller gain.
   *
   * @param Kp proportional coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setP(double Kp) {
    m_thisMutex.lock();
    try {
      m_Kp = Kp;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the Integral coefficient of the PID controller gain.
   *
   * @param Ki integral coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setI(double Ki) {
    m_thisMutex.lock();
    try {
      m_Ki = Ki;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the Differential coefficient of the PID controller gain.
   *
   * @param Kd differential coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setD(double Kd) {
    m_thisMutex.lock();
    try {
      m_Kd = Kd;
    } finally {
      m_thisMutex.unlock();
    }
  }

  public void setF(double Kf) {
    m_thisMutex.lock();
    try {
      m_Kf = Kf;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Get the Proportional coefficient.
   *
   * @return proportional coefficient
   */
  public double getP() {
    m_thisMutex.lock();
    try {
      return m_Kp;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Get the Integral coefficient.
   *
   * @return integral coefficient
   */
  public double getI() {
    m_thisMutex.lock();
    try {
      return m_Ki;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Get the Differential coefficient.
   *
   * @return differential coefficient
   */
  public double getD() {
    m_thisMutex.lock();
    try {
      return m_Kd;
    } finally {
      m_thisMutex.unlock();
    }
  }

  public double getF() {
    m_thisMutex.lock();
    try {
      return m_Kf;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Returns the period of this controller.
   *
   * @return the period of the controller.
   */
  public double getPeriod() {
    m_thisMutex.lock();
    try {
      return m_period;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the setpoint for the PIDController.
   *
   * @param setpoint The desired setpoint.
   */
  public void setSetpoint(double setpoint) {
    m_thisMutex.lock();
    try {
      if (m_maximumInput > m_minimumInput) {
        m_setpoint = MathUtils.clamp(setpoint, m_minimumInput, m_maximumInput);
      } else {
        m_setpoint = setpoint;
      }
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Returns the current setpoint of the PIDController.
   *
   * @return The current setpoint.
   */
  public double getSetpoint() {
    m_thisMutex.lock();
    try {
      return m_setpoint;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Returns true if the error is within the percentage of the total input range, determined by
   * SetTolerance. This asssumes that the maximum and minimum input were set using SetInput.
   *
   * <p>This will return false until at least one input value has been computed.
   *
   * @return Whether the error is within the acceptable bounds.
   */
  public boolean atSetpoint() {
    m_thisMutex.lock();
    try {
      return Math.abs(m_positionError) < m_positionTolerance
          && Math.abs(m_velocityError) < m_velocityTolerance;
    } finally { 
      m_thisMutex.unlock();
    }
  }

  /**
   * Enables continuous input.
   *
   * <p>Rather then using the max and min input range as constraints, it considers
   * them to be the same point and automatically calculates the shortest route
   * to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_thisMutex.lock();
    try {
      m_continuous = true;
      setInputRange(minimumInput, maximumInput);
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Disables continuous input.
   */
  public void disableContinuousInput() {
    m_thisMutex.lock();
    try {
      m_continuous = false;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the minimum and maximum values for the integrator.
   *
   * <p>When the cap is reached, the integrator value is added to the controller
   * output rather than the integrator value times the integral gain.
   *
   * @param minimumIntegral The minimum value of the integrator.
   * @param maximumIntegral The maximum value of the integrator.
   */
  public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
    m_thisMutex.lock();
    try {
      m_minimumIntegral = minimumIntegral;
      m_maximumIntegral = maximumIntegral;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   */
  public void setTolerance(double positionTolerance) {
    setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   */
  public void setTolerance(double positionTolerance, double velocityTolerance) {
    m_thisMutex.lock();
    try {
      m_positionTolerance = positionTolerance;
      m_velocityTolerance = velocityTolerance;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Returns the difference between the setpoint and the measurement.
   *
   * @return The error.
   */
  public double getPositionError() {
    m_thisMutex.lock();
    try {
      return getContinuousError(m_positionError);
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Returns the velocity error.
   */
  public double getVelocityError() {
    m_thisMutex.lock();
    try {
      return m_velocityError;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   */
  protected void calculate() {
    if (m_origSource == null || m_pidOutput == null) {
      return;
    }

    boolean enabled;
    
    m_thisMutex.lock();
    try {
      enabled = m_enabled;
    } finally {
      m_thisMutex.unlock();
    }

    if (enabled) {
      double input;

      double P;
      double I;
      double D;
      double F;

      double period;

      double maximumIntegral;
      double minimumIntegral;

      // Storage for function input-outputs
      double positionError;
      double velocityError;
      double totalError;

      m_thisMutex.lock();
      try {
        input = m_pidSource.getAsDouble();

        P = m_Kp;
        I = m_Ki;
        D = m_Kd;
        F = m_Kf;

        period = m_period;

        maximumIntegral = m_maximumIntegral;
        minimumIntegral = m_minimumIntegral;

        totalError = m_totalError;
        positionError = getContinuousError(m_setpoint - input);
        velocityError = (m_positionError - m_prevError) / m_period;

        
      } finally {
        m_thisMutex.unlock();
      }

      // Storage for function outputs
      double result;

      if (I != 0) {
        totalError = MathUtils.clamp(totalError + positionError * period,
            minimumIntegral / I, maximumIntegral / I);
      }

      result = calculateProportional(P, positionError)
              + calculateIntegral(I, totalError)
              + calculateDerivative(D, velocityError) + F;

      // Ensures m_enabled check and pidWrite() call occur atomically
      m_pidOutputMutex.lock();
      try {
        m_thisMutex.lock();
        try {
          if (m_enabled) {
            // Don't block other PIDController operations on pidWrite()
            m_thisMutex.unlock();

            m_pidOutput.accept(result);
          }
        } finally {
          if (m_thisMutex.isHeldByCurrentThread()) {
            m_thisMutex.unlock();
          }
        }
      } finally {
        m_pidOutputMutex.unlock();
      }

      m_thisMutex.lock();
      try {
        m_prevError = positionError;
        m_totalError = totalError;
        m_result = result;
        m_positionError = positionError;
        m_velocityError = velocityError;
      } finally {
        m_thisMutex.unlock();
      }
    }
  }


  protected synchronized double calculateProportional(double p, double error) {
    return p*error;
  }
  
  protected synchronized double calculateIntegral(double i, double totalerror) {
    return i*totalerror;
  }
  
  protected synchronized double calculateDerivative(double d, double derror) {
    return d*derror;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.setSafeState(this::reset);
    builder.addDoubleProperty("p", this::getP, this::setP);
    builder.addDoubleProperty("i", this::getI, this::setI);
    builder.addDoubleProperty("d", this::getD, this::setD);
    builder.addDoubleProperty("f", this::getF, this::setF);
    builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    builder.addBooleanProperty("enabled", this::isEnabled, this::setEnabled);
  }

  /**
   * Wraps error around for continuous inputs. The original error is returned if continuous mode is
   * disabled.
   *
   * @param error The current error of the PID controller.
   * @return Error for continuous inputs.
   */
  protected double getContinuousError(double error) {
    m_thisMutex.lock();
    try {
      if (m_continuous && m_inputRange > 0) {
        error %= m_inputRange;
        if (Math.abs(error) > m_inputRange / 2) {
          if (error > 0) {
              return error - m_inputRange;
            } else {
              return error + m_inputRange;
            }
          }
        }
        return error;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the minimum and maximum values expected from the input.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  private void setInputRange(double minimumInput, double maximumInput) {
    this.m_thisMutex.lock();
    try {
      m_minimumInput = minimumInput;
      m_maximumInput = maximumInput;
      m_inputRange = maximumInput - minimumInput;

      // Clamp setpoint to new input
      if (m_maximumInput > m_minimumInput) {
        m_setpoint = MathUtils.clamp(m_setpoint, m_minimumInput, m_maximumInput);
      }

    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Return the current PID result This is always centered on zero and constrained the the max and
   * min outs.
   *
   * @return the latest calculated output
   */
  public double get() {
    m_thisMutex.lock();
    try {
      return m_result;
    } finally {
      m_thisMutex.unlock();
    }
  }

  public void reset() {
    disable();

    m_thisMutex.lock();
    try {
      m_prevError = 0;
      m_totalError = 0;
      m_result = 0;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Begin running the PIDController.
   */
  private void enable() {
    m_thisMutex.lock();
    try {
      m_enabled = true;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Stop running the PIDController, this sets the output to zero before stopping.
   */
  private void disable() {
    // Ensures m_enabled check and pidWrite() call occur atomically
    m_pidOutputMutex.lock();
    try {
      m_thisMutex.lock();
      try {
        m_enabled = false;
      } finally {
        m_thisMutex.unlock();
      }

      m_pidOutput.accept(0);
    } finally {
      m_pidOutputMutex.unlock();
    }
  }
  /**
   * Set the enabled state of the PIDController.
   */
  public void setEnabled(boolean enable) {
    if (enable) {
      enable();
    } else {
      disable();
    }
  }

  /**
   * Return true if PIDController is enabled.
   */
  private boolean isEnabled() {
    m_thisMutex.lock();
    try {
      return m_enabled;
    } finally {
      m_thisMutex.unlock();
    }
  }
}
