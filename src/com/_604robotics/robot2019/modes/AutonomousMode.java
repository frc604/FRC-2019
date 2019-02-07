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

	private class FallBackMacro extends StatefulCoordinator {
		public FallBackMacro() {
			super(FallBackMacro.class);
			
			addState("Pathfind back 144in", new PathStraight( PathFinderUtil.inchesToMeters( 144 ), true ));
		}
	}

	private class FallForwardMacro extends StatefulCoordinator {
		public FallForwardMacro() {
			super(FallForwardMacro.class);
			
			addState("Pathfind forward 144in", new PathStraight( PathFinderUtil.inchesToMeters( 144 ) ));
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
			addState("", new PathFollower( new Waypoint[] {
				new Waypoint( 0, 0, 0 ),
				new Waypoint( PathFinderUtil.feetToMeters(14), 0, 0 ),
			} ));
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

	private class PathFollower extends Coordinator {
		private Trajectory path;
		private EncoderFollower leftFollower;
		private EncoderFollower rightFollower;
		private java.util.Timer timer;
		private SmartTimer timeElapsed;
		private Drive.TankDrive tankDrive;
		private Pulse PIDTargetPulse=new Pulse();

		public PathFollower( Waypoint[] waypoints ) {
			this( Pathfinder.generate( waypoints, Calibration.PATHFINDER_CONFIG ), false );
		}

		public PathFollower( Waypoint[] waypoints, boolean reversePath ) {
			this( Pathfinder.generate( waypoints, Calibration.PATHFINDER_CONFIG ), reversePath );
		}

		public PathFollower( Trajectory path ) {
			this( path, false );
		}

		public PathFollower( Trajectory path, boolean reverseDrive ) {
			this.path = path;
			timeElapsed = new SmartTimer();
			tankDrive = robot.drive.new TankDrive( false );
			generateTankPath( reverseDrive );
		}

		private void generateTankPath( boolean reverseDrive ) {
			TankModifier modifier = new TankModifier(path).modify( Calibration.Pathfinder.ROBOT_WIDTH, Calibration.PATHFINDER_CONFIG );

			if( reverseDrive ) {
				leftFollower = new EncoderFollower( Pathfinder.reverseTrajectory( modifier.getLeftTrajectory( ) ) );
				rightFollower = new EncoderFollower( Pathfinder.reverseTrajectory( modifier.getRightTrajectory() ) );
			} else {
				leftFollower = new EncoderFollower( modifier.getLeftTrajectory() );
				rightFollower = new EncoderFollower( modifier.getRightTrajectory() );
			}
		}

		class PathFollowTask extends TimerTask {
			private double prevAngleError=0;
			private PathFollowSide side;
			private double next_dt = 0;

			private double clamp(double value, double low, double high) {
				return Math.max(low, Math.min(value, high));
			}

			PathFollowTask (PathFollowSide side) {
				this.side=side;
			}

			@Override
			public void run() {
				tankDrive.activate();
				int encoderPos;
				double rawPow;
				Trajectory.Segment prev_seg;
				Trajectory.Segment seg;
				if (side==PathFollowSide.LEFT) {
					encoderPos = robot.drive.leftClicks.get();
					rawPow = leftFollower.calculate(encoderPos);
					prev_seg = leftFollower.prevSegment();
					seg = leftFollower.getSegment();
				} else {//if side==PathFollowSide.RIGHT
					encoderPos=robot.drive.rightClicks.get();
					rawPow = rightFollower.calculate(encoderPos);
					prev_seg = rightFollower.prevSegment();
					seg = rightFollower.getSegment();
				}

				next_dt = seg.dt;

				// Calculated curvature scales with velocity
				// Keeping old implied scaling since faster movement implies more curvature correction
				// Use harmonic mean because curvature is 1/radius
				double scaleVel=2*leftFollower.getSegment().velocity*rightFollower.getSegment().velocity;
				if (scaleVel!=0) {
					scaleVel/=(leftFollower.getSegment().velocity+rightFollower.getSegment().velocity);
				}
				double curvature = PathFinderUtil.getScaledCurvature(prev_seg,seg,scaleVel);

				double normcurv=PathFinderUtil.getNormalizedCurvature(prev_seg, seg);
				//System.out.println("Normcurv is "+normcurv+" for "+side.toString());

				// Raw heading stuff here due to side selections
				double degreeHeading = AutonMovement.clicksToDegrees(Calibration.DRIVE_PROPERTIES,
						robot.drive.leftClicks.get()-robot.drive.rightClicks.get());
				//System.out.print("Current heading is "+degreeHeading);

				// Both headings are the same
				double desiredHeading = leftFollower.getHeading();
				desiredHeading=Pathfinder.r2d(desiredHeading);
				desiredHeading=Pathfinder.boundHalfDegrees(desiredHeading);
				desiredHeading*=-1;
				// Pathfinder heading is counterclockwise math convention
				// We are using positive=right clockwise convention

				double angleError = desiredHeading-degreeHeading;
				System.out.println("Angle error is "+angleError);

				// Convert back into radians for consistency
				angleError = Pathfinder.d2r(angleError);
				double dAngleError=angleError-prevAngleError;
				dAngleError/=seg.dt;

				double kappa_val=Calibration.Pathfinder.K_KAPPA*curvature;
				double pTheta_val=Calibration.Pathfinder.K_PTHETA_0/(Calibration.Pathfinder.K_PTHETA_DECAY*normcurv*normcurv+1);
				pTheta_val*=angleError;
				double dTheta_val=Calibration.Pathfinder.K_DTHETA_0/(Calibration.Pathfinder.K_DTHETA_DECAY*normcurv*normcurv+1);
				dTheta_val*=dAngleError;

				dTheta_val=clamp(dTheta_val,-pTheta_val,pTheta_val);

				/*
				double deshed=-Pathfinder.boundHalfRadians(leftFollower.getHeading());
				System.out.println("Equal "+(deshed-Pathfinder.d2r(desiredHeading)));
				*/

				if (side==PathFollowSide.LEFT) {
					tankDrive.leftPower.set(rawPow+kappa_val
							+pTheta_val+dTheta_val);
				} else { // RIGHT side
					tankDrive.rightPower.set(rawPow-kappa_val
							-pTheta_val-dTheta_val);
				}
				prevAngleError=angleError;

				if( side==PathFollowSide.LEFT ) {
					timer.schedule( new PathFollowTask( PathFollowSide.LEFT), (long) ( 1000*getNextdt() ) );
				} else if( side == PathFollowSide.RIGHT ) {
					timer.schedule( new PathFollowTask( PathFollowSide.RIGHT), (long) (1000*getNextdt()) );
				}
			}

			double getNextdt() {
				return next_dt;
			}
		}


		@Override
		public void begin() {
			System.out.println("ERROR: Begin");
			timer = new java.util.Timer();
			robot.drive.resetSensors();
			System.out.println("ERROR: left is "+robot.drive.leftClicks.get());
			leftFollower.configurePIDVA(Calibration.Pathfinder.KP, Calibration.Pathfinder.KI, Calibration.Pathfinder.KD,
					Calibration.Pathfinder.KV, Calibration.Pathfinder.KA);
			rightFollower.configurePIDVA(Calibration.Pathfinder.KP, Calibration.Pathfinder.KI, Calibration.Pathfinder.KD,
					Calibration.Pathfinder.KV, Calibration.Pathfinder.KA);
			leftFollower.configureEncoder(0, 250, 0.12732); // 5 in diameter
			rightFollower.configureEncoder(0, 250, 0.12732);
			leftFollower.reset();
			rightFollower.reset();
			timer.schedule(new PathFollowTask(PathFollowSide.LEFT), 0);
			timer.schedule(new PathFollowTask(PathFollowSide.RIGHT), 0);
			tankDrive.activate();
			timeElapsed.start();
		}

		@Override
		public boolean run() {
			tankDrive.activate();
			return timeElapsed.runUntil(0.6, new Runnable() {
				@Override
				public void run() {
					boolean targetReached = (leftFollower.isFinished() && rightFollower.isFinished());
					if (!targetReached) {
						timeElapsed.reset();
						PIDTargetPulse.update(true);
					} else {
						PIDTargetPulse.update(false);
					}
					if (PIDTargetPulse.isFallingEdge()) {
						System.out.println("========Finished========");
					}
				}
			});
		}

		@Override
		public void end() {
			timer.cancel();
			System.out.println("ERROR: end");
			leftFollower.reset();
			rightFollower.reset();
			timeElapsed.stopAndReset();
		}
	}

	private class PathStraight extends PathFollower {
		public PathStraight( double meters ) {
			this( meters, false );
		}

		public PathStraight( double meters, boolean reverseDrive ) {
			super( Pathfinder.generate( new Waypoint[]{
					new Waypoint( 0,0,0 ),
					new Waypoint( meters, 0, 0 ),
			}, Calibration.PATHFINDER_CONFIG ), reverseDrive );
		}
	}


}
