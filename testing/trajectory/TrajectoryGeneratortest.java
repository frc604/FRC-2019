/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package trajectory;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import com._604robotics.robotnik.utils.geometry.Pose2d;
import com._604robotics.robotnik.utils.geometry.Rotation2d;
import com._604robotics.robotnik.utils.geometry.Transform2d;
import com._604robotics.robotnik.utils.geometry.Translation2d;
import com._604robotics.robotnik.utils.trajectory.constraint.TrajectoryConstraint;
import com._604robotics.robotnik.utils.trajectory.Trajectory;
import com._604robotics.robotnik.utils.trajectory.TrajectoryGenerator;
import com._604robotics.robotnik.utils.PathFinderUtil;


import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertTrue;


class TrajectoryGeneratorTest {
  static Trajectory getTrajectory(List<TrajectoryConstraint> constraints) {
    final double startVelocity = 0;
    final double endVelocity = 0;
    final double maxVelocity = PathFinderUtil.feetToMeters(12.0);
    final double maxAccel = PathFinderUtil.feetToMeters(12);

    // 2018 cross scale auto waypoints.
    var sideStart = new Pose2d(PathFinderUtil.feetToMeters(1.54), PathFinderUtil.feetToMeters(23.23),
        Rotation2d.fromDegrees(-180));
    var crossScale = new Pose2d(PathFinderUtil.feetToMeters(23.7), PathFinderUtil.feetToMeters(6.8),
        Rotation2d.fromDegrees(-160));

    var waypoints = new ArrayList<Pose2d>();
    waypoints.add(sideStart);
    waypoints.add(sideStart.plus(
        new Transform2d(new Translation2d(PathFinderUtil.feetToMeters(-13), PathFinderUtil.feetToMeters(0)),
            new Rotation2d())));
    waypoints.add(sideStart.plus(
        new Transform2d(new Translation2d(PathFinderUtil.feetToMeters(-19.5), PathFinderUtil.feetToMeters(5)),
            Rotation2d.fromDegrees(-90))));
    waypoints.add(crossScale);

    return TrajectoryGenerator.generateTrajectory(
        waypoints,
        constraints,
        startVelocity,
        endVelocity,
        maxVelocity,
        maxAccel,
        true
    );
  }

  @Test
  @SuppressWarnings("LocalVariableName")
  void testGenerationAndConstraints() {
    Trajectory trajectory = getTrajectory(new ArrayList<>());

    double duration = trajectory.getTotalTimeSeconds();
    double t = 0.0;
    double dt = 0.02;

    while (t < duration) {
      var point = trajectory.sample(t);
      t += dt;
      assertAll(
          () -> assertTrue(Math.abs(point.velocityMetersPerSecond) < PathFinderUtil.feetToMeters(12.0) + 0.05),
          () -> assertTrue(Math.abs(point.accelerationMetersPerSecondSq) < PathFinderUtil.feetToMeters(12.0)
              + 0.05)
      );
    }
  }
}