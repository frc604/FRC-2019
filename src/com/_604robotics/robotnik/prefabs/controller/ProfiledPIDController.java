package com._604robotics.robotnik.prefabs.controller;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpiutil.math.MathUtil;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Subclass of ProfiledPIDController that has a feedforward for rotating arms. This subclass
 * requires an PIDSource as the PIDSource and uses only continuous error. Zero is assumed to be
 * horizontal. Users are responsible for properly zeroing the PIDSource beforehand.
 */
public class ProfiledPIDController extends NewExtendablePIDController {
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  // Motion Profile constraints.
  private TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints();

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, Kd, and motion profile
   * constrains and a default period of 0.02 seconds with no feedforward.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param constraints The motion profile constraints.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public ProfiledPIDController(
      double Kp,
      double Ki,
      double Kd,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {
    this(Kp, Ki, Kd, 0.0, constraints, 0.02, pidSource, pidOutput);
  }

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, Kd, motion profile
   * constrains, and period with no feedforward.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param constraints The motion profile constraints.
   * @param period The period of the controller.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public ProfiledPIDController(
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
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, Kd, motion profile
   * constrains, and period with no feedforward.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param Kf The feedforward term.
   * @param constraints The motion profile constraints.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public ProfiledPIDController(
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
  public ProfiledPIDController(
      double Kp,
      double Ki,
      double Kd,
      double Kf,
      TrapezoidProfile.Constraints constraints,
      double period,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {

    super(Kp, Ki, Kd, Kf, period, pidSource, pidOutput);
    this.m_constraints = constraints;
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

  @Override
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
        profile = new TrapezoidProfile(m_constraints, m_goal, this.m_setpoint);
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
            MathUtil.clamp(
                totalError + positionError * period, minimumIntegral / I, maximumIntegral / I);
      }

      // Calculating the result
      result =
          calculateProportional(P, positionError)
              + calculateIntegral(I, totalError)
              + calculateDerivative(D, velocityError)
              + calculateFeedForward();

      // Clamping the result
      result = MathUtil.clamp(result, minimumOutput, maximumOutput);

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

  /**
   * Sets the minimum and maximum values expected from the input.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  @Override
  public void setInputRange(double minimumInput, double maximumInput) {
    this.m_thisMutex.lock();
    try {
      m_minimumInput = minimumInput;
      m_maximumInput = maximumInput;
      m_inputRange = maximumInput - minimumInput;

      // Clamp setpoint to new input
      if (m_maximumInput > m_minimumInput) {
        m_setpoint =
            new TrapezoidProfile.State(
                MathUtil.clamp(m_setpoint.position, m_minimumInput, m_maximumInput),
                m_setpoint.velocity);
      }

    } finally {
      m_thisMutex.unlock();
    }
  }

  @Override
  public double getSetpoint() {
    throw new UnsupportedOperationException("Should not be accessed by, use getGoal() instead!");
  }

  @Override
  public void setSetpoint(double setpoint) {
    throw new UnsupportedOperationException("Should not be accessed by, use setGoal() instead!");
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
    builder.addBooleanProperty("enabled", this::isEnabled, this::setEnabled);
  }
}
