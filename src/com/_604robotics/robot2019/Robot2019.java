package com._604robotics.robot2019;

import com._604robotics.robot2019.modes.*;
import com._604robotics.robot2019.modules.*;
import com._604robotics.robot2019.modules.Flywheel;
import com._604robotics.robot2019.systems.*;
import com._604robotics.robotnik.Robot;
import com._604robotics.robotnik.prefabs.modules.*;
import edu.wpi.first.wpilibj.RobotBase;

public class Robot2019 extends Robot {

  public static void main(String[] args) {
    RobotBase.startRobot(Robot2019::new);
  }

  public final Dashboard dashboard = addModule(new Dashboard());
  public final DashboardSystem dashboardSystem =
      addSystem(DashboardSystem.class, new DashboardSystem(this));
  public final Flywheel flywheel = addModule(new Flywheel());

  public final TeleopMode teleopMode = setTeleopMode(new TeleopMode(this));
  public final AutonomousMode autonomousMode = setAutonomousMode(new AutonomousMode(this));
}
