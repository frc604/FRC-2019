/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package kinematics;

import org.junit.jupiter.api.Test;

import com._604robotics.robotnik.utils.geometry.Rotation2d;
import com._604robotics.robotnik.utils.kinematics.ChassisSpeeds;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

class ChassisSpeedsTest {
  private static final double kEpsilon = 1E-9;

  @Test
  void testFieldRelativeConstruction() {
    final var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        1.0, 0.0, 0.5, Rotation2d.fromDegrees(-90.0)
    );

    assertAll(
        () -> assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, kEpsilon),
        () -> assertEquals(1.0, chassisSpeeds.vyMetersPerSecond, kEpsilon),
        () -> assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kEpsilon)
    );
  }
}