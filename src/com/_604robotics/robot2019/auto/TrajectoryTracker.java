package com._604robotics.robot2019.auto;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.modules.Drive;
import com._604robotics.robotnik.Coordinator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class TrajectoryTracker extends Coordinator {
  private Drive.TankDriveVolts m_tank;
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;
  private Drive m_drivetrain;
  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private final RamseteController m_controller;
  private final SimpleMotorFeedforward m_feedforward;
  private final PIDController m_leftVelController;
  private final PIDController m_rightVelController;
  private double m_prevTime;

  public TrajectoryTracker(Trajectory trajectory, Drive drivetrain) {
    this.m_trajectory = trajectory;
    this.m_drivetrain = drivetrain;
    m_controller = new RamseteController(Calibration.Auto.RAMSETE_B, Calibration.Auto.RAMSETE_ZETA);
    m_feedforward =
        new SimpleMotorFeedforward(
            Calibration.Auto.KS_VOLTS,
            Calibration.Auto.KV_VOLT_SECONDS_PER_METER,
            Calibration.Auto.KA_VOLT_SECONDS_SQUARED_PER_METER);
    m_leftVelController = new PIDController(Calibration.Auto.KP_DRIVE_VELCOTIY, 0, 0);
    m_rightVelController = new PIDController(Calibration.Auto.KP_DRIVE_VELCOTIY, 0, 0);
  }

  @Override
  public void begin() {
    m_prevTime = 0;
    var initialState = m_trajectory.sample(0);
    m_prevSpeeds =
        m_drivetrain.m_driveKinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();
    m_leftVelController.reset();
    m_rightVelController.reset();
  }

  @Override
  public boolean run() {
    System.out.println(m_drivetrain.getPose());
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    var targetWheelSpeeds =
        m_drivetrain.m_driveKinematics.toWheelSpeeds(
            m_controller.calculate(m_drivetrain.getPose(), m_trajectory.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    double leftFeedforward =
        m_feedforward.calculate(
            leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

    double rightFeedforward =
        m_feedforward.calculate(
            rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

    leftOutput =
        leftFeedforward
            + m_leftVelController.calculate(
                m_drivetrain.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);

    rightOutput =
        rightFeedforward
            + m_rightVelController.calculate(
                m_drivetrain.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);

    m_tank = m_drivetrain.new TankDriveVolts();

    m_tank.leftVolts.set(leftOutput);
    m_tank.rightVolts.set(rightOutput);

    m_tank.activate();

    m_prevTime = curTime;
    m_prevSpeeds = targetWheelSpeeds;

    return !m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
  }

  @Override
  public void end() {
    m_timer.stop();
    m_tank.leftVolts.set(0.0);
    m_tank.rightVolts.set(0.0);
    m_tank.activate();
  }
}
