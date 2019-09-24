/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package trajectory;

import java.util.Collections;

import org.junit.jupiter.api.Test;

import com._604robotics.robotnik.utils.kinematics.ChassisSpeeds;
import com._604robotics.robotnik.utils.kinematics.DifferentialDriveKinematics;
import com._604robotics.robotnik.utils.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import com._604robotics.robotnik.utils.PathFinderUtil;
import com._604robotics.robotnik.utils.trajectory.Trajectory;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DifferentialDriveKinematicsConstraintTest {
  @SuppressWarnings({"LocalVariableName", "PMD.AvoidInstantiatingObjectsInLoops"})
  @Test
  void testDifferentialDriveKinematicsConstraint() {
    double maxVelocity = PathFinderUtil.feetToMeters(12.0); // 12 feet per second
    var kinematics = new DifferentialDriveKinematics(PathFinderUtil.inchesToMeters(27));
    var constraint = new DifferentialDriveKinematicsConstraint(kinematics, maxVelocity);

    Trajectory trajectory = TrajectoryGeneratorTest.getTrajectory(
        Collections.singletonList(constraint));

    var duration = trajectory.getTotalTimeSeconds();
    var t = 0.0;
    var dt = 0.02;

    while (t < duration) {
      var point = trajectory.sample(t);
      var chassisSpeeds = new ChassisSpeeds(
          point.velocityMetersPerSecond, 0,
          point.velocityMetersPerSecond * point.curvatureRadPerMeter
      );

      var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

      t += dt;
      assertAll(
          () -> assertTrue(wheelSpeeds.leftMetersPerSecond <= maxVelocity + 0.05),
          () -> assertTrue(wheelSpeeds.rightMetersPerSecond <= maxVelocity + 0.05)
      );
    }
  }
}