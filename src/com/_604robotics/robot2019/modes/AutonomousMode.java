package com._604robotics.robot2019.modes;

import java.io.IOException;
import java.util.TimerTask;

import com._604robotics.marionette.InputRecording;
import com._604robotics.robot2019.Robot2019;
import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.coordinators.SleepCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.SwitchCoordinator;
import com._604robotics.robotnik.prefabs.flow.Pulse;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import com._604robotics.robotnik.utils.AutonMovement;
import com._604robotics.robotnik.utils.PathFinderUtil;
import com._604robotics.robotnik.utils.annotations.Unreal;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class AutonomousMode extends Coordinator {
	private static final Logger logger = new Logger(AutonomousMode.class);

	private final com._604robotics.robot2019.Robot2019 robot;

	private Coordinator selectedModeMacro;

	public String primaryFileName;
	public String secondaryFileName;

	public static enum PathFollowSide {
		LEFT,
		RIGHT
	}

	public AutonomousMode (Robot2019 robot) {
		this.robot = robot;
	}

	@Override
	public void begin () {
		robot.teleopMode.execute();
	}

	@Override
	public void end () {
		robot.teleopMode.stop();
	}

}
