package com._604robotics.robot2019.modes;

import com._604robotics.robot2019.Robot2019;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;

public class AutonomousMode extends Coordinator {
  private static final Logger logger = new Logger(AutonomousMode.class);

  private final com._604robotics.robot2019.Robot2019 robot;

  private Coordinator selectedModeMacro;

  public AutonomousMode(Robot2019 robot) {
    this.robot = robot;
  }

  @Override
  public void begin() {}

  @Override
  public boolean run() {
    return true;
  }

  @Override
  public void end() {}
}
