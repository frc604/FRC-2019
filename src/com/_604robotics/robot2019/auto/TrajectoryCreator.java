package com._604robotics.robot2019.auto;

import java.util.List;

import com._604robotics.robot2019.constants.Calibration;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

public class TrajectoryCreator {
    private DifferentialDriveKinematics m_kinematics;
    private TrajectoryConstraint[] m_constraints;

    public TrajectoryCreator(DifferentialDriveKinematics kinematics, TrajectoryConstraint... constraints) {
        m_constraints = constraints;
        m_kinematics = kinematics;
    }


    public Trajectory getTrajectory(List<Pose2d> waypoints, boolean reverse) {
        var config = new TrajectoryConfig(Calibration.Auto.MAX_SPEED_METERS_PER_SECOND, Calibration.Auto.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        config.setKinematics(m_kinematics);
        
        for (TrajectoryConstraint i: m_constraints){
            config.addConstraint(i);
        }

        if ( reverse ) {
            config.setReversed(true);
        } else {
            config.setReversed(false);
        }

        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

}