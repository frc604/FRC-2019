package com._604robotics.robot2019.modes;

import java.util.List;

import com._604robotics.robot2019.Robot2019;
import com._604robotics.robot2019.auto.TrajectoryCreator;
import com._604robotics.robot2019.auto.TrajectoryTracker;
import com._604robotics.robot2019.macros.ArcadeTimedDriveMacro;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.coordinators.SleepCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

import static com._604robotics.robot2019.constants.Calibration.Auto.MAX_SPEED_METERS_PER_SECOND;
import static com._604robotics.robot2019.constants.Calibration.Auto.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
import static com._604robotics.robot2019.constants.Calibration.Auto.MAX_CENTRIPETAL_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
import static com._604robotics.robot2019.constants.Calibration.Auto.KS_VOLTS;
import static com._604robotics.robot2019.constants.Calibration.Auto.KV_VOLT_SECONDS_PER_METER;
import static com._604robotics.robot2019.constants.Calibration.Auto.KA_VOLT_SECONDS_SQUARED_PER_METER;


public class AutonomousMode extends Coordinator {
	private static final Logger logger = new Logger(AutonomousMode.class);
	
	private final com._604robotics.robot2019.Robot2019 robot;

	private Coordinator selectedModeMacro;

	private TrajectoryCreator trajectoryCreator;


	public AutonomousMode (Robot2019 robot) {
		this.robot = robot;
		System.out.println("ONCE");

		trajectoryCreator = new TrajectoryCreator(robot.drive.m_driveKinematics,
			new DifferentialDriveKinematicsConstraint(robot.drive.m_driveKinematics, MAX_SPEED_METERS_PER_SECOND),
			new DifferentialDriveVoltageConstraint( new SimpleMotorFeedforward(
				KS_VOLTS,
				KV_VOLT_SECONDS_PER_METER,
				KA_VOLT_SECONDS_SQUARED_PER_METER),
				robot.drive.m_driveKinematics,
				10),
			new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCELERATION_RADIANS_PER_SECOND_SQUARED));
	}

	@Override
	public void begin () {
		System.out.println(robot.dashboard.autonMode.get());
		switch (robot.dashboard.autonMode.get()) {
			case MANUAL:
				selectedModeMacro = robot.teleopMode;
				break;
			case FAILSAFE_FORWARD_12:
				selectedModeMacro = new FallForwardMacro();
				break;
			case FAILSAFE_BACKWARD_12:
				selectedModeMacro = new FallBackMacro();
				break;
			case DEMO_NEW_AUTON:
				selectedModeMacro = new DemoStateMacro();
				break;
			case OFF:
			default:
				System.out.println("huh");
				selectedModeMacro = null;
				break;
		}

		System.out.println(selectedModeMacro);

		if (selectedModeMacro != null) {
			selectedModeMacro.start();
		}
	}

	@Override
	public boolean run () {
		if (selectedModeMacro == null) {
			return false;
		}
		System.out.println(robot.dashboard.autonMode.get());
		robot.drive.updateOdometry();
		return selectedModeMacro.execute();
	}

	@Override
	public void end () {
		if (selectedModeMacro != null) {
			selectedModeMacro.stop();
		}
	}


	private class FallBackMacro extends StatefulCoordinator {
		public FallBackMacro() {
			super(FallBackMacro.class);
			
			addState("Pathfind back 144in", new PathReverse( Units.inchesToMeters( 36 ) ));
		}
	}

	private class FallForwardMacro extends StatefulCoordinator {
		public FallForwardMacro() {
			super(FallForwardMacro.class);
			
			addState("Pathfind forward 144in", new PathStraight( Units.inchesToMeters( 36 ) ));
		}
	}

	private class TimedMacro extends ArcadeTimedDriveMacro {
		protected double movePower;
		protected double rotatePower;
		protected double time;

		public TimedMacro( double movePower, double rotatePower, double time ) {
			super(robot);
			this.movePower = movePower;
			this.rotatePower = rotatePower;
			this.time = time;
		}

		@Override
		protected double getMovePower() {
			return movePower;
		}

		@Override
		protected double getRotatePower() {
			return rotatePower;
		}

		@Override
		protected double getTime() {
			return time;
		}
	}

	private class DemoStateMacro extends StatefulCoordinator {
		public DemoStateMacro() {
			super(DemoStateMacro.class);

			addState("", new TrajectoryTracker(
			trajectoryCreator.getTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(1.5, -1, new Rotation2d(0)),
				new Pose2d(3, 0, new Rotation2d(Math.PI/2))
			), false),
			robot.drive));
			addState("Waiting...", new SleepCoordinator(3));
			addState("", new TrajectoryTracker(
			trajectoryCreator.getTrajectory(
			List.of(
				robot.drive.getPose(),
				new Pose2d(1.5, -0.5, Rotation2d.fromDegrees(90)),
				new Pose2d(0, 0, new Rotation2d(0))
			), true),
			robot.drive));
		}
	}

	private class PathStraight extends TrajectoryTracker {

		public PathStraight( double meters ) {
			super(trajectoryCreator.getTrajectory(
			List.of(
				new Pose2d(),
				new Pose2d(meters, 0, new Rotation2d(0))
			), false),
			robot.drive);

		}
	}

	private class PathReverse extends TrajectoryTracker {

		public PathReverse( double meters ) {
			super(trajectoryCreator.getTrajectory(
			List.of(
				new Pose2d(),
				new Pose2d(-meters, 0, new Rotation2d(0))
			), true),
			robot.drive);

		}
	}


}
