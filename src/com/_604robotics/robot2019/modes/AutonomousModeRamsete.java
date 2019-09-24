package com._604robotics.robot2019.modes;

import java.io.IOException;
import java.util.TimerTask;

import com._604robotics.marionette.InputRecording;
import com._604robotics.robot2019.Robot2019;
import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.macros.ArcadeTimedDriveMacro;
import com._604robotics.robot2019.modules.Drive;
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
import com._604robotics.robotnik.utils.geometry.Rotation2d;
import com._604robotics.robotnik.utils.geometry.Pose2d;
import com._604robotics.robotnik.utils.kinematics.ChassisSpeeds;
import com._604robotics.robotnik.utils.kinematics.DifferentialDriveKinematics;
import com._604robotics.robotnik.utils.kinematics.DifferentialDriveOdometry;
import com._604robotics.robotnik.utils.kinematics.DifferentialDriveWheelSpeeds;
import com._604robotics.robotnik.prefabs.controller.RamseteController;
import com._604robotics.robotnik.prefabs.controller.SingleThreadPIDController;
import com._604robotics.robotnik.utils.trajectory.constraint.TrajectoryConstraint;
import com._604robotics.robotnik.utils.trajectory.Trajectory;
import com._604robotics.robotnik.utils.trajectory.TrajectoryGenerator;
import com._604robotics.robotnik.utils.trajectory.Trajectory.State;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

import java.util.ArrayList;
import java.util.List;

public class AutonomousModeRamsete extends Coordinator {
	private static final Logger logger = new Logger(AutonomousModeRamsete.class);

	private final com._604robotics.robot2019.Robot2019 robot;

	private Coordinator selectedModeMacro;

	public String primaryFileName;
	public String secondaryFileName;

	public AutonomousModeRamsete (Robot2019 robot) {
		this.robot = robot;
	}

	@Override
	public void begin () {
		// Filename is prefixed in MarionetteDriver
		primaryFileName = robot.dashboard.marionetteFile.get();
		secondaryFileName = "";

		Coordinator marionetteDriver;

		switch( robot.dashboard.marionetteOutput.get() ) {
			case MANUAL:
				marionetteDriver = new MarionetteDriver(primaryFileName);
				break;
			default:
				marionetteDriver = new FallForwardMacro();
				break;
		}

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
			case MARIONETTE:
				selectedModeMacro = marionetteDriver;
				break;
			case OFF:
			default:
				selectedModeMacro = null;
				break;
		}

		if (selectedModeMacro != null) {
			selectedModeMacro.start();
		}
	}

	@Override
	public boolean run () {
		if (selectedModeMacro == null) {
			return false;
		}

		return selectedModeMacro.execute();
	}

	@Override
	public void end () {
		if (selectedModeMacro != null) {
			selectedModeMacro.stop();
		}
	}

	private class CustomMarionetteDriver extends Coordinator {
		private String fileName;

		public CustomMarionetteDriver(String fileName) {
			this.fileName = fileName;
		}

		@Override
		protected void begin () {
			logger.info("Loading Marionette recording from \"" + fileName + "\"");
			final InputRecording recording;
			try {
				recording = InputRecording.load("/home/lvuser/" + fileName);
			} catch (IOException e) {
				throw new RuntimeException(e);
			}

			logger.info("Starting Marionette playback");
			robot.teleopMode.startPlayback(recording);
			robot.teleopMode.start();
		}

		@Override
		protected boolean run () {
			return robot.teleopMode.execute();
		}

		@Override
		protected void end () {
			logger.info("Stopping Marionette playback");
			robot.teleopMode.stop();
			robot.teleopMode.stopPlayback();
		}
	}

	private class MarionetteDriver extends Coordinator {
		private String fileName;

		public MarionetteDriver(String fileName) {
			this.fileName = robot.dashboard.filePrefix.get() + fileName;
		}

		@Override
		protected void begin () {
			logger.info("Loading Marionette recording from \"" + robot.dashboard.filePrefix.get() + fileName + "\"");
			final InputRecording recording;
			try {
				recording = InputRecording.load("/home/lvuser/" + robot.dashboard.filePrefix.get() + fileName);
			} catch (IOException e) {
				throw new RuntimeException(e);
			}

			logger.info("Starting Marionette playback");
			robot.teleopMode.startPlayback(recording);
			robot.teleopMode.start();
		}

		@Override
		protected boolean run () {
			return robot.teleopMode.execute();
		}

		@Override
		protected void end () {
			logger.info("Stopping Marionette playback");
			robot.teleopMode.stop();
			robot.teleopMode.stopPlayback();
		}
	}

	protected final class TankOdometryHandler {
		private Drive.TankDrive tankDrive = robot.drive.new TankDrive(false);
		
		private SingleThreadPIDController leftController;
		private SingleThreadPIDController rightController;

		private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Calibration.DRIVE_PROPERTIES.trackWidth);

 	 	private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(kinematics);


		public TankOdometryHandler() {
			tankDrive.activate();

			leftController = new SingleThreadPIDController(1, 0, 0);
			rightController = new SingleThreadPIDController(1, 0, 0);
		}

		public double getLeftError() {
			return leftController.getPositionError();
		}

		public double getRightError() {
			return rightController.getPositionError();
		}

		public Rotation2d getAngle() {
			return Rotation2d.fromDegrees(-robot.drive.gyroAngle.get());
		}

		public DifferentialDriveWheelSpeeds getCurrentSpeeds() {
			return new DifferentialDriveWheelSpeeds(robot.drive.leftClickRate.get(), robot.drive.rightClickRate.get());

		}

		public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
			double leftOutput = leftController.calculate(robot.drive.leftClickRate.get(),
				speeds.leftMetersPerSecond);
			double rightOutput = rightController.calculate(robot.drive.rightClickRate.get(),
				speeds.rightMetersPerSecond);

			tankDrive.leftPower.set(leftOutput);
			tankDrive.rightPower.set(rightOutput);

		}

		public void drive(double speed, double rot) {
			DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0, rot));
			setSpeeds(wheelSpeeds);
		}

		public Pose2d updateOdometry(double time) {
			return odometry.updateWithTime(time, getAngle(), getCurrentSpeeds());
		}

		public void reset(Pose2d pose){
			odometry.resetPosition(pose);
		}

	}

	private class PathFollower extends Coordinator {
		private Trajectory path;
		private Drive.TankDrive tankDrive;
		private RamseteController controller;
		private PathFollowTask followTask;


		public PathFollower(List<Pose2d> waypoints, List<TrajectoryConstraint> constraints, double startVelocity, double endVelocity, double maxVelocity, double maxAccel, double b, double zeta, double dt) {
			this(TrajectoryGenerator.generateTrajectory(
				waypoints,
				constraints,
				startVelocity,
				endVelocity,
				maxVelocity,
				maxAccel,
				true
			));
			this.dt = dt;
			followTask = new PathFollowTask(b, zeta, dt);
		}

		public PathFollower(Trajectory path) {
			this.path = path;
			tankDrive = robot.drive.new TankDrive( false );

		}

		class PathFollowTask extends TimerTask {
			private State robotState;
			private State desiredState;
			private Pose2d robotPose;
			private Pose2d desiredPose;
			private ChassisSpeeds output;
			private TankOdometryHandler tank;
			private double dt;
			private double intitialTime;


			PathFollowTask(double b, double zeta, double dt) {
				this.dt = dt;
				controller = new RamseteController(b, zeta);
				tank = new TankOdometryHandler();
				tank.reset(path.sample(0).poseMeters);
				intitialTime = Timer.getFPGATimestamp();

			}

		

			@Override
			public void run() {
				tankDrive.activate();
				robotPose = tank.updateOdometry(Timer.getFPGATimestamp()-intitialTime);
				
				desiredState = path.sample((Timer.getFPGATimestamp()-intitialTime) + dt);
				desiredPose = desiredState.poseMeters;
			
				output = controller.calculate(robotPose, desiredState);

				tank.drive(output.vxMetersPerSecond, output.omegaRadiansPerSecond);
				
			}

			public double getTimePassed() {
				return (Timer.getFPGATimestamp()-intitialTime);
			}

		}

		@Override
		public void begin() {
			robot.drive.resetSensors();
			
		}

		public boolean run() {
			
		}
		
	}
}
