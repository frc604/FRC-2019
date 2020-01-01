package com._604robotics.robot2019.systems;

import com._604robotics.robotnik.Coordinator;

public class DashboardSystem extends Coordinator {
  private final com._604robotics.robot2019.Robot2019 robot;

  public DashboardSystem(com._604robotics.robot2019.Robot2019 robot) {
    this.robot = robot;
  }

  @Override
  public boolean run() {

    // Leave at end of function
    return true;
  }
}
