/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package controller;

import org.junit.jupiter.api.Test;

import com._604robotics.robotnik.utils.geometry.Pose2d;
import com._604robotics.robotnik.utils.geometry.Rotation2d;
import com._604robotics.robotnik.utils.geometry.Twist2d;
import com._604robotics.robotnik.utils.trajectory.Trajectory;

import com._604robotics.robotnik.prefabs.controller.RamseteController;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

class RamseteControllerTest {
  private final double[][] m_trajectory = SampleTrajectory.getInstance().getPath();
  private static final double kTolerance = 1 / 12.0;
  private static final double kAngularTolerance = Math.toRadians(2);

  @Test
  @SuppressWarnings("PMD.AvoidInstantiatingObjectsInLoops")
  void testReachesReference() {
    final var controller = new RamseteController(2.0, 0.7);

    var robotPose = new Pose2d(2.7, 23.0, Rotation2d.fromDegrees(0.0));

    for (int i = 0; i <= m_trajectory.length - 1; ++i) {
      double[] trajectoryPoint = m_trajectory[i];
      double dt = trajectoryPoint[0];

      var desiredPose = new Pose2d(trajectoryPoint[1], trajectoryPoint[2],
          new Rotation2d(trajectoryPoint[3]));
      var desiredV = trajectoryPoint[4];

      double desiredOmega;
      if (i == m_trajectory.length - 1) {
        desiredOmega = 0.0;
      } else {
        desiredOmega = boundRadians((m_trajectory[i + 1][3] - trajectoryPoint[3]) / dt);
      }

      double curvature = 0.0;
      if (desiredV != 0.0) {
        curvature = desiredOmega / desiredV;
      }
      var desiredState = new Trajectory.State(0.0, desiredV, 0.0, desiredPose, curvature);
      var output = controller.calculate(robotPose, desiredState);
      robotPose = robotPose.exp(new Twist2d(output.vxMetersPerSecond * dt, 0,
          output.omegaRadiansPerSecond * dt));
    }

    Pose2d finalRobotPose = robotPose;
    double expectedX = m_trajectory[m_trajectory.length - 1][1];
    double expectedY = m_trajectory[m_trajectory.length - 1][2];
    double expectedTheta = m_trajectory[m_trajectory.length - 1][3];
    assertAll(
        () -> assertEquals(expectedX, finalRobotPose.getTranslation().getX(), kTolerance),
        () -> assertEquals(expectedY, finalRobotPose.getTranslation().getY(), kTolerance),
        () -> assertEquals(0.0,
            boundRadians(expectedTheta - finalRobotPose.getRotation().getRadians()),
            kAngularTolerance)
    );
  }

  private static double boundRadians(double value) {
    while (value > Math.PI) {
      value -= Math.PI * 2;
    }
    while (value <= -Math.PI) {
      value += Math.PI * 2;
    }
    return value;
  }
}