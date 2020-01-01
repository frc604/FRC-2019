package com._604robotics.robotnik.prefabs.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import java.util.List;

public class TrajectoryCreator {
  private DifferentialDriveKinematics m_kinematics;
  private TrajectoryConstraint[] m_constraints;

  private double m_maxSpeed;
  private double m_maxAcceleration;

  public TrajectoryCreator(
      DifferentialDriveKinematics kinematics,
      TrackerConstants constants,
      TrajectoryConstraint... constraints) {
    m_constraints = constraints;
    m_kinematics = kinematics;
    m_maxSpeed = constants.maxSpeed;
    m_maxAcceleration = constants.maxAcceleration;
  }

  public Trajectory getTrajectory(List<Pose2d> waypoints, boolean reverse) {
    var config = new TrajectoryConfig(m_maxSpeed, m_maxAcceleration);
    config.setKinematics(m_kinematics);

    for (TrajectoryConstraint i : m_constraints) {
      config.addConstraint(i);
    }

    if (reverse) {
      config.setReversed(true);
    } else {
      config.setReversed(false);
    }

    return TrajectoryGenerator.generateTrajectory(waypoints, config);
  }
}
