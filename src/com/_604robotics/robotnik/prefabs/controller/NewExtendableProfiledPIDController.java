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

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpiutil.math.MathUtils;
import java.util.TimerTask;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/** Implements a Motion Profiled PID control loop. */
public class NewExtendableProfiledPIDController implements Sendable, AutoCloseable {
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

  // Maximum integral value
  private double m_maximumIntegral = 1.0;

  // Minimum integral value
  private double m_minimumIntegral = -1.0;

  // Maximum input - limit setpoint to this
  private double m_maximumInput;

  // Minimum input - limit setpoint to this
  private double m_minimumInput;

  private double m_maximumOutput = 1;
  private double m_minimumOutput = -1;

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

  // Last calculated result of the controller
  private double m_result = 0.0;

  // Goal and setpoint states for the PIDController.
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  // Motion Profile constraints.
  private TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints();

  // Output consumer.
  protected DoubleConsumer m_pidOutput;

  // Source supplier.
  protected DoubleSupplier m_pidSource;

  // Original source of the controller, if it changes.
  private DoubleSupplier m_origSource;

  // Thread lock for the controller.
  ReentrantLock m_thisMutex = new ReentrantLock();

  // Thread lock for writing to the output.
  ReentrantLock m_pidOutputMutex = new ReentrantLock();

  // Control loop timer, used to schedule the controller calculate though a TimerTask.
  java.util.Timer m_controlLoop;

  // Timer task used to schedule controller.
  private class PIDTask extends TimerTask {
    private NewExtendableProfiledPIDController m_controller;

    PIDTask(NewExtendableProfiledPIDController controller) {
      requireNonNullParam(controller, "controller", "Given PIDController was null");

      m_controller = controller;
    }

    @Override
    public void run() {
      // Running calculate on measurement and setpoint
      m_controller.calculate();
    }
  }

  /**
   * Allocates a NewExtendableProfiledPIDController with the given constants for Kp, Ki, Kd, and
   * motion profile constrains and a default period of 0.02 seconds with no feedforward.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param constraints The motion profile constraints.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public NewExtendableProfiledPIDController(
      double Kp,
      double Ki,
      double Kd,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {
    this(Kp, Ki, Kd, 0.0, constraints, 0.02, pidSource, pidOutput);
  }

  /**
   * Allocates a NewExtendableProfiledPIDController with the given constants for Kp, Ki, Kd, motion
   * profile constrains, and period with no feedforward.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param constraints The motion profile constraints.
   * @param period The period of the controller.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public NewExtendableProfiledPIDController(
      double Kp,
      double Ki,
      double Kd,
      TrapezoidProfile.Constraints constraints,
      double period,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {
    this(Kp, Ki, Kd, 0.0, constraints, period, pidSource, pidOutput);
  }

  /**
   * Allocates a NewExtendableProfiledPIDController with the given constants for Kp, Ki, Kd, motion
   * profile constrains, and period with no feedforward.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param Kf The feedforward term.
   * @param constraints The motion profile constraints.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public NewExtendableProfiledPIDController(
      double Kp,
      double Ki,
      double Kd,
      double Kf,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {
    this(Kp, Ki, Kd, Kf, constraints, 0.02, pidSource, pidOutput);
  }

  /**
   * Allocates a NewExtendablePIDController with the given constants for Kp, Ki, Kd, Kf, and period.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param Kf The feedforward term.
   * @param constraints The motion profile constraints.
   * @param period The period of the controller in seconds.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidSource A lambda consuming the controller output.
   */
  public NewExtendableProfiledPIDController(
      double Kp,
      double Ki,
      double Kd,
      double Kf,
      TrapezoidProfile.Constraints constraints,
      double period,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {

    // Making sure null supplier and consumers are not passed in.
    requireNonNullParam(constraints, "constraints", "NewExtendableProfiledPIDContoller");
    requireNonNullParam(pidSource, "source", "NewExtendableProfiledPIDContoller");
    requireNonNullParam(pidOutput, "output", "NewExtendableProfiledPIDContoller");

    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;

    m_Kf = Kf;

    m_period = period;

    m_constraints = constraints;

    m_pidSource = pidSource;
    m_pidOutput = pidOutput;

    m_origSource = pidSource;

    m_controlLoop = new java.util.Timer();

    // Scheduling controller calculate
    m_controlLoop.schedule(new PIDTask(this), 0L, (long) (m_period * 1000));

    instances++;
    SendableRegistry.addLW(this, "ProfiledPIDController", instances);

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

  /**
   * Sets the Feedforward term.
   *
   * @param Kf feedforward term
   */
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

  /**
   * Get the Feedforward term.
   *
   * @return feedforward term
   */
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
   * @param goal The desired setpoint.
   */
  public void setGoal(TrapezoidProfile.State goal) {
    m_thisMutex.lock();
    try {
      m_goal = goal;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the goal for the PIDController.
   *
   * @param goal The goal position.
   */
  public void setGoal(double goal) {
    m_thisMutex.lock();
    try {
      m_goal = new TrapezoidProfile.State(goal, 0);
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Returns the current setpoint of the PIDController.
   *
   * @return The current setpoint.
   */
  public TrapezoidProfile.State getGoal() {
    m_thisMutex.lock();
    try {
      return m_goal;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Returns true if the error is within the percentage of the total input range, determined by
   * SetTolerance. This assumes that the maximum and minimum input were set using SetInput.
   *
   * <p>This will return false until at least one input value has been computed.
   *
   * @return Whether the error is within the acceptable bounds.
   */
  public boolean atGoal() {
    m_thisMutex.lock();
    try {
      return atSetpoint() && m_goal.equals(m_setpoint);
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the constraints for the controller motion profile.
   *
   * @param constraints The constraints
   */
  public void setConstraints(TrapezoidProfile.Constraints constraints) {
    m_thisMutex.lock();
    try {
      m_constraints = constraints;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Returns the set constraints for the controller motion profile.
   *
   * @return The motion profile constraints.
   */
  public TrapezoidProfile.Constraints getConstraints() {
    m_thisMutex.lock();
    try {
      return m_constraints;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Returns true if the error is within the percentage of the total input range, determined by
   * SetTolerance. This assumes that the maximum and minimum input were set using SetInput.
   *
   * <p>This will return false until at least one input value has been computed.
   *
   * <p>Does not return if the MotionProfile is at the end use {@link this#atGoal()} for that.
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
   * <p>Rather then using the max and min input range as constraints, it considers them to be the
   * same point and automatically calculates the shortest route to the setpoint.
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

  /** Disables continuous input. */
  public void disableContinuousInput() {
    m_thisMutex.lock();
    try {
      m_continuous = false;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the minimum and maximum values to write.
   *
   * @param minimumOutput the minimum percentage to write to the output
   * @param maximumOutput the maximum percentage to write to the output
   */
  public void setOutputRange(double minimumOutput, double maximumOutput) {
    m_thisMutex.lock();
    try {
      if (minimumOutput > maximumOutput) {
        throw new BoundaryException("Lower bound is greater than upper bound");
      }
      m_minimumOutput = minimumOutput;
      m_maximumOutput = maximumOutput;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the minimum and maximum values for the integrator.
   *
   * <p>When the cap is reached, the integrator value is added to the controller output rather than
   * the integrator value times the integral gain.
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

  /** Returns the velocity error. */
  public double getVelocityError() {
    m_thisMutex.lock();
    try {
      return m_velocityError;
    } finally {
      m_thisMutex.unlock();
    }
  }

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

      double period;

      double maximumIntegral;
      double minimumIntegral;

      double maximumOutput;
      double minimumOutput;

      TrapezoidProfile profile;
      TrapezoidProfile.State setpoint;

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

        period = m_period;

        maximumIntegral = m_maximumIntegral;
        minimumIntegral = m_minimumIntegral;

        maximumOutput = m_maximumOutput;
        minimumOutput = m_minimumOutput;

        totalError = m_totalError;

        // Making the motion profile and finding the controller setpoint.
        profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        setpoint = profile.calculate(m_period);

        positionError = getContinuousError(setpoint.position - input);
        velocityError = (positionError - m_prevError) / m_period;

      } finally {
        m_thisMutex.unlock();
      }

      // Storage for function outputs
      double result;

      // Clamping the Integral coefficient
      if (I != 0) {
        totalError =
            MathUtils.clamp(
                totalError + positionError * period, minimumIntegral / I, maximumIntegral / I);
      }

      // Calculating the result
      result =
          calculateProportional(P, positionError)
              + calculateIntegral(I, totalError)
              + calculateDerivative(D, velocityError)
              + calculateFeedForward();

      // Clamping the result
      result = MathUtils.clamp(result, minimumOutput, maximumOutput);

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
        m_setpoint = setpoint;
        m_result = result;
        m_positionError = positionError;
        m_velocityError = velocityError;
      } finally {
        m_thisMutex.unlock();
      }
    }
  }

  // Separating calculate for each term so controller can be extended
  protected synchronized double calculateProportional(double p, double error) {
    return p * error;
  }

  protected synchronized double calculateIntegral(double i, double totalerror) {
    return i * totalerror;
  }

  protected synchronized double calculateDerivative(double d, double derror) {
    return d * derror;
  }

  protected synchronized double calculateFeedForward() {
    m_thisMutex.lock();
    try {
      return m_Kf;
    } finally {
      m_thisMutex.unlock();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.setSafeState(this::reset);
    builder.addDoubleProperty("p", this::getP, this::setP);
    builder.addDoubleProperty("i", this::getI, this::setI);
    builder.addDoubleProperty("d", this::getD, this::setD);
    builder.addDoubleProperty("f", this::getF, this::setF);
    builder.addDoubleProperty("goal", () -> getGoal().position, this::setGoal);
    builder.addDoubleProperty(
        "maxVelocity",
        () -> getConstraints().maxVelocity,
        (maxVelocity) ->
            setConstraints(
                new TrapezoidProfile.Constraints(maxVelocity, getConstraints().maxAcceleration)));
    builder.addDoubleProperty(
        "maxAcceleration",
        () -> getConstraints().maxAcceleration,
        (maxAcceleration) ->
            setConstraints(
                new TrapezoidProfile.Constraints(getConstraints().maxVelocity, maxAcceleration)));
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
        m_setpoint =
            new TrapezoidProfile.State(
                MathUtils.clamp(m_setpoint.position, m_minimumInput, m_maximumInput),
                m_setpoint.velocity);
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

  /** Begin running the PIDController. */
  private void enable() {
    m_thisMutex.lock();
    try {
      m_enabled = true;
    } finally {
      m_thisMutex.unlock();
    }
  }

  /** Stop running the PIDController, this sets the output to zero before stopping. */
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
  /** Set the enabled state of the PIDController. */
  public void setEnabled(boolean enable) {
    if (enable) {
      enable();
    } else {
      disable();
    }
  }

  /** Return true if PIDController is enabled. */
  private boolean isEnabled() {
    m_thisMutex.lock();
    try {
      return m_enabled;
    } finally {
      m_thisMutex.unlock();
    }
  }
}
