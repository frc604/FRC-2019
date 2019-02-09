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
import com._604robotics.robotnik.utils.annotations.Unreal;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AutonomousMode extends Coordinator {
	private static final Logger logger = new Logger(AutonomousMode.class);

	private final com._604robotics.robot2019.Robot2019 robot;

	private Coordinator selectedModeMacro;
	private TeleopMode selectedTeleopMode;

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
				selectedTeleopMode = robot.teleopMode;
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
				selectedTeleopMode = null;
				break;
		}

		if (selectedTeleopMode != null) {
			selectedTeleopMode.start();
		} else if (selectedModeMacro != null) {
			selectedModeMacro.start();
		}
	}
	@Override
	public boolean run() {
		if( selectedTeleopMode == null && selectedModeMacro == null ) {
			return false;
		} else if( selectedModeMacro != null ) {
			return selectedModeMacro.execute();
		}

		return selectedTeleopMode.execute();
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
			System.out.println("Executing teleop mode uwu");
			return robot.teleopMode.execute();
		}

		@Override
		protected void end () {
			logger.info("Stopping Marionette playback");
			robot.teleopMode.stop();
			robot.teleopMode.stopPlayback();
		}
	}

	private class FallBackMacro extends StatefulCoordinator {
		public FallBackMacro() {
			super(FallBackMacro.class);
			
			//addState("Pathfind back 144in", new PathStraight( PathFinderUtil.inchesToMeters( 144 ), true ));
		}
	}

	private class FallForwardMacro extends StatefulCoordinator {
		public FallForwardMacro() {
			super(FallForwardMacro.class);
			
			//addState("Pathfind forward 144in", new PathStraight( PathFinderUtil.inchesToMeters( 144 ) ));
		}
	}

	protected final class ArcadePIDCoordinator extends Coordinator {
		private Logger arcadePIDLog=new Logger(ArcadePIDCoordinator.class);
		// Timer that is started to continue running PID for some time after equilibrium
		private SmartTimer timeElapsed = new SmartTimer();
		private Drive.ArcadeDrive arcadeDrive = robot.drive.new ArcadeDrive(false);
		private Pulse PIDTargetPulse=new Pulse();
		// PIDOutputs write to persistent Input that retain their values
		// This prevents jerky movement when PIDs don't run often enough
		// Rotation PIDOutput
		private PIDOutput rotateBot = new PIDOutput() {
			@Override
			public synchronized void pidWrite(double output) {
				arcadeDrive.rotatePower.set(output);
			}
		};
		// Move PIDOutput
		private PIDOutput moveBot = new PIDOutput() {
			@Override
			public synchronized void pidWrite(double output) {
				arcadeDrive.movePower.set(output);
			}
		};
		// Encoder rotation PIDSource
		private PIDSource encoderDiff = new PIDSource() {

			private PIDSourceType type;

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {type = pidSource;}

			@Override
			public PIDSourceType getPIDSourceType() {return type;}

			@Override
			public double pidGet() {
				return -robot.drive.rightClicks.get()+robot.drive.leftClicks.get();
			}
		};
		// Encoder moving PIDSource
		private PIDSource encoderAvg = new PIDSource() {

			private PIDSourceType type;

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {type = pidSource;}

			@Override
			public PIDSourceType getPIDSourceType() {return type;}

			@Override
			public double pidGet() {
				return (robot.drive.rightClicks.get()+robot.drive.leftClicks.get())/2;
			}

		};
		// Declaration of PIDControllers
		// Avoid initialization here because parameters require tweaking as well
		private PIDController rotController;
		private PIDController moveController;
		private double moveSetpoint;
		private double rotSetpoint;

		public ArcadePIDCoordinator(double moveSetpoint, double rotSetpoint) {
			super();
			this.moveSetpoint = moveSetpoint;
			this.rotSetpoint = rotSetpoint;
		}

		public double getMoveError() {
			return moveController.getError();
		}

		public double getRotError() {
			return rotController.getError();
		}

		@Override
		protected void begin() {
			// Activate arcadeDrive and reset encoder and gyro
			arcadeDrive.activate();
			robot.drive.resetSensors();
			// Set up PIDSource details
			encoderDiff.setPIDSourceType(PIDSourceType.kDisplacement);
			encoderAvg.setPIDSourceType(PIDSourceType.kDisplacement);
			// Set up rotation PID controller
			rotController = new PIDController(Calibration.DRIVE_ROTATE_PID_P,
					Calibration.DRIVE_ROTATE_PID_I,
					Calibration.DRIVE_ROTATE_PID_D,
					encoderDiff,
					rotateBot,
					Calibration.DRIVE_PID_SAMPLE_RATE);
			rotController.setSetpoint(rotSetpoint);
			rotController.setOutputRange(-Calibration.DRIVE_ROTATE_PID_MAX,
					Calibration.DRIVE_ROTATE_PID_MAX);
			rotController.setAbsoluteTolerance(Calibration.DRIVE_ROTATE_TOLERANCE);
			// Set up move PID controller
			moveController = new PIDController(Calibration.DRIVE_MOVE_PID_P,
					Calibration.DRIVE_MOVE_PID_I,
					Calibration.DRIVE_MOVE_PID_D,
					encoderAvg,
					moveBot,
					Calibration.DRIVE_PID_SAMPLE_RATE);
			moveController.setSetpoint(moveSetpoint);
			moveController.setOutputRange(-Calibration.DRIVE_MOVE_PID_MAX,
					Calibration.DRIVE_MOVE_PID_MAX);
			moveController.setAbsoluteTolerance(Calibration.DRIVE_MOVE_TOLERANCE);
			arcadePIDLog.log("INFO", "Enabling rotation controller");
			rotController.enable();
			// Stagger the timings of the PIDs slightly
			try {
				// 500 = 1000 / 2
				// Set up PIDs to output in even staggering
				Thread.sleep((long) (Calibration.DRIVE_PID_SAMPLE_RATE*500));
			} catch (InterruptedException e) {
				// Do nothing
			}
			arcadePIDLog.log("INFO", "Enabling move controller");
			moveController.enable();
			// Instead of using complex logic to start timer,
			// Start timer here and constantly reset until setpoint is reached
			timeElapsed.start();
			PIDTargetPulse.update(true);
		}

		@Override
		protected synchronized boolean run() {
			arcadeDrive.activate();
			// System.out.println("Move error is " + getMoveError() + ", Rot error is " + getRotError());
			return timeElapsed.runUntil(Calibration.DRIVE_PID_AFTER_TIMING, new Runnable() {
				@Override
				public void run() {
					boolean targetReached = rotController.onTarget() && moveController.onTarget();
					if (!targetReached) {
						timeElapsed.reset();
						PIDTargetPulse.update(true);
					} else {
						PIDTargetPulse.update(false);
					}
					if (PIDTargetPulse.isFallingEdge()) {
						arcadePIDLog.log("INFO", "Target reached, enabling timer");
					} else if (PIDTargetPulse.isRisingEdge()) {
						arcadePIDLog.log("INFO", "Target left, disabling timer");
					}
				}
			});
		}

		@Override
		protected void end() {
			arcadePIDLog.log("INFO","Final Rotate error is "+rotController.getError());
			arcadePIDLog.log("INFO","Final Move error is "+moveController.getError());
			rotController.disable();
			moveController.disable();
			timeElapsed.stopAndReset();
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
			/*addState("", new PathFollower( new Waypoint[] {
				new Waypoint( 0, 0, 0 ),
				new Waypoint( PathFinderUtil.feetToMeters(14), 0, 0 ),
			} ));*/
		}
	}

	private class ArcadePIDStateMacro extends StatefulCoordinator {
		private Logger arcadePIDLog=new Logger("ArcadePIDStateMacro");
		private ArcadePIDCoordinator controller;
		public ArcadePIDStateMacro (double moveSetpoint, double rotSetpoint) {
			super(ArcadePIDStateMacro.class);
			arcadePIDLog.log("INFO", "Move Setpoint is "+moveSetpoint);
			arcadePIDLog.log("INFO", "Rotate setpoint is "+rotSetpoint);
			// Set up a rotate class with both Move and Rotate PIDs
			// Instead of just setting a difference, actively move forwards/backwards to compensate for REAL life
			// This avoids COMPLEX imperfections and reduces many issues to be purely IMAGINARY
			controller = new ArcadePIDCoordinator(moveSetpoint, rotSetpoint);
			addState("ArcadePID",controller);
		}
	}
}
