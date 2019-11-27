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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpiutil.math.MathUtils;

import java.util.TimerTask;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * Implements a PID control loop.
 */
public class NewExtendableProfiliedPIDController implements Sendable {
  private static int instances;

  private PIDController m_controller;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.Constraints m_constraints;

  // Is the pid controller enabled
  private boolean m_enabled = false;

  // The percentage or absolute error that is considered at setpoint.
  private double m_positionTolerance = 0.05;
  private double m_velocityTolerance = Double.POSITIVE_INFINITY;

  private double m_Kf = 0;

  private double m_result = 0.0;

  private DoubleConsumer m_pidOutput;
  private DoubleSupplier m_pidSource;

  private DoubleSupplier m_origSource;

  ReentrantLock m_thisMutex = new ReentrantLock();
  ReentrantLock m_pidOutputMutex = new ReentrantLock();

  java.util.Timer m_controlLoop;
  Timer m_setpointTimer;

  private class PIDTask extends TimerTask {
    private NewExtendableProfiliedPIDController m_controller;

    PIDTask(NewExtendableProfiliedPIDController controller) {
      requireNonNullParam(controller, "controller", "Given PIDController was null");

      m_controller = controller;
    }

    @Override
    public void run() {
      m_controller.calculate();
    }
  }

  public NewExtendableProfiliedPIDController(double Kp, double Ki, double Kd,
    TrapezoidProfile.Constraints constraints, DoubleSupplier source, DoubleConsumer output) {
    this(Kp, Ki, Kd, 0.0, constraints, 0.02, source, output);
  }

  public NewExtendableProfiliedPIDController(double Kp, double Ki, double Kd,
    TrapezoidProfile.Constraints constraints, double period,  DoubleSupplier source, DoubleConsumer output) {
    this(Kp, Ki, Kd, 0.0, constraints, period, source, output);
  }

  /**
   * Allocates a NewExtendablePIDController with the given constants for Kp, Ki,
   * and Kd and a default period of 0.02 seconds.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param Kf The feedforward term.
   */
  public NewExtendableProfiliedPIDController(double Kp, double Ki, double Kd, double period,
    TrapezoidProfile.Constraints constraints, DoubleSupplier source, DoubleConsumer output) {
    this(Kp, Ki, Kd, 0.0, constraints,period,  source, output);
  }

  /**
   * Allocates a NewExtendablePIDController with the given constants for Kp, Ki,
   * and Kd.
   *
   * @param Kp     The proportional coefficient.
   * @param Ki     The integral coefficient.
   * @param Kd     The derivative coefficient.
   * @param period The period between controller updates in seconds.
   */
  public NewExtendableProfiliedPIDController(double Kp, double Ki, double Kd, double Kf,
      TrapezoidProfile.Constraints constraints, double period, DoubleSupplier source, DoubleConsumer output) {
    requireNonNullParam(constraints, "contraints", "NewExtendableProfiledPIDContoller");
    requireNonNullParam(source, "source", "NewExtendableProfiledPIDContoller");
    requireNonNullParam(output, "output", "NewExtendableProfiledPIDContoller");

    m_controller = new PIDController(Kp, Ki, Kd, period);

    m_pidSource = source;
    m_pidOutput = output;

    m_origSource = source;

    m_controlLoop = new java.util.Timer();
    m_setpointTimer = new Timer();
    m_setpointTimer.start();

    m_controlLoop.schedule(new PIDTask(this), 0L, (long) (m_controller.getPeriod() * 1000));

    instances++;
    SendableRegistry.addLW(this, "PIDController", instances);

    HAL.report(tResourceType.kResourceType_PIDController, instances);
  }

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
   * <p>
   * Set the proportional, integral, and differential coefficients.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   */
  @SuppressWarnings("ParameterName")
  public void setPID(double Kp, double Ki, double Kd) {
    m_thisMutex.lock();
    try {
      m_controller.setPID(Kp, Ki, Kd);
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
      m_controller.setP(Kp);
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
      m_controller.setI(Ki);
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
      m_controller.setD(Kd);
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
      return m_controller.getP();
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
      return m_controller.getI();
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
      return m_controller.getD();
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
      return m_controller.getPeriod();
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the goal for the PIDController.
   *
   * @param setpoint The goal state.
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
   * @param setpoint The goal position.
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
   * Returns true if the error is within the percentage of the total input range,
   * determined by SetTolerance. This asssumes that the maximum and minimum input
   * were set using SetInput.
   *
   * <p>
   * This will return false until at least one input value has been computed.
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
   * Returns true if the error is within the percentage of the total input range,
   * determined by SetTolerance. This asssumes that the maximum and minimum input
   * were set using SetInput.
   *
   * <p>
   * This will return false until at least one input value has been computed.
   *
   * @return Whether the error is within the acceptable bounds.
   */
  public boolean atSetpoint() {
    m_thisMutex.lock();
    try {
      return m_controller.atSetpoint();
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Enables continuous input.
   *
   * <p>
   * Rather then using the max and min input range as constraints, it considers
   * them to be the same point and automatically calculates the shortest route to
   * the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_thisMutex.lock();
    try {
      m_controller.enableContinuousInput(minimumInput, maximumInput);
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
      m_controller.disableContinuousInput();
    } finally {
      m_thisMutex.unlock();
    }
  }

  /**
   * Sets the minimum and maximum values for the integrator.
   *
   * <p>
   * When the cap is reached, the integrator value is added to the controller
   * output rather than the integrator value times the integral gain.
   *
   * @param minimumIntegral The minimum value of the integrator.
   * @param maximumIntegral The maximum value of the integrator.
   */
  public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
    m_thisMutex.lock();
    try {
      m_controller.setIntegratorRange(minimumIntegral, maximumIntegral);
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
      m_controller.setTolerance(positionTolerance, velocityTolerance);
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
      return m_controller.getPositionError();
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
      return m_controller.getPositionError();
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

      TrapezoidProfile.Constraints constraints;
      TrapezoidProfile.State goal;
      TrapezoidProfile.State setpoint;

      double result;

      m_thisMutex.lock();
      try {
        input = m_pidSource.getAsDouble();


        constraints = m_constraints;
        goal = m_goal;
        setpoint = m_setpoint;

        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        setpoint = profile.calculate(m_controller.getPeriod());
        
        result = m_controller.calculate(input, setpoint.position);
      } finally {
        m_thisMutex.unlock();
      }

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
        m_result = result;
      } finally {
        m_thisMutex.unlock();
      }
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ProfiledPIDController");
    builder.addDoubleProperty("p", this::getP, this::setP);
    builder.addDoubleProperty("i", this::getI, this::setI);
    builder.addDoubleProperty("d", this::getD, this::setD);
    builder.addDoubleProperty("goal", () -> getGoal().position, this::setGoal);
    builder.addBooleanProperty("enabled", this::isEnabled, this::setEnabled);
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
      m_result = 0;
      m_controller.reset();
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
