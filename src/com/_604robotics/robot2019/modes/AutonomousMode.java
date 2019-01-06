package com._604robotics.robot2019.modes;

import com._604robotics.robot2019.Robot2017;
import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.modules.Drive;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.coordinators.SleepCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;
import com._604robotics.robotnik.prefabs.flow.Pulse;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import com._604robotics.robotnik.utils.AutonMovement;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AutonomousMode extends Coordinator {
    private final Robot2017 robot;

    private final Coordinator middleAutonModeMacro;

    private final Coordinator blueLeftAutonModeMacro;
    private final Coordinator blueRightAutonModeMacro;

    private final Coordinator redLeftAutonModeMacro;
    private final Coordinator redRightAutonModeMacro;
    
    private final Coordinator rotateLeftStateMacro;
    private final Coordinator rotateRightStateMacro;
    private final Coordinator forwardStateMacro;
    private final Coordinator forwardEmpericalMacro;
    private final Coordinator backwardStateMacro;
    private final Coordinator rotateFullLeftStateMacro;
    
    private Coordinator selectedModeMacro;

    public AutonomousMode (Robot2017 robot) {
        this.robot = robot;

        middleAutonModeMacro = new MiddleAutonModeMacro();

        blueLeftAutonModeMacro = new SideAutonModeMacro("BlueLeftAutonModeMacro") {
            @Override
            protected double getDriveForwardClicks () { return robot.dashboard.blueLeftAutonDriveForwardClicks.get(); }
            @Override
            protected double getTurnToFacePegAngle () { return robot.dashboard.blueLeftAutonTurnToFacePegAngle.get(); }
        };

        blueRightAutonModeMacro = new SideAutonModeMacro("BlueRightAutonModeMacro") {
            @Override
            protected double getDriveForwardClicks () { return robot.dashboard.blueRightAutonDriveForwardClicks.get(); }
            @Override
            protected double getTurnToFacePegAngle () { return robot.dashboard.blueRightAutonTurnToFacePegAngle.get(); }
        };

        redLeftAutonModeMacro = new SideAutonModeMacro("RedLeftAutonModeMacro") {
            @Override
            protected double getDriveForwardClicks () { return robot.dashboard.redLeftAutonDriveForwardClicks.get(); }
            @Override
            protected double getTurnToFacePegAngle () { return robot.dashboard.redLeftAutonTurnToFacePegAngle.get(); }
        };

        redRightAutonModeMacro = new SideAutonModeMacro("RedRightAutonModeMacro") {
            @Override
            protected double getDriveForwardClicks () { return robot.dashboard.redRightAutonDriveForwardClicks.get(); }
            @Override
            protected double getTurnToFacePegAngle () { return robot.dashboard.redRightAutonTurnToFacePegAngle.get(); }
        };
                
        rotateLeftStateMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_STILL_TARGET,
                Calibration.DRIVE_ROTATE_LEFT_TARGET);
        rotateRightStateMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_STILL_TARGET,
                Calibration.DRIVE_ROTATE_RIGHT_TARGET);
        rotateFullLeftStateMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_STILL_TARGET,
        		AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 360*10));
        forwardStateMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_FORWARD_TARGET,
                Calibration.DRIVE_ROTATE_STILL_TARGET);
        forwardEmpericalMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_FORWARD_TEST_INCHES, 0);
        backwardStateMacro = new ArcadePIDStateMacro(Calibration.DRIVE_MOVE_BACKWARD_TARGET,
                Calibration.DRIVE_ROTATE_STILL_TARGET);
    }

    @Override
    public void begin () {
        switch (robot.dashboard.autonMode.get()) {
            case MIDDLE:
                selectedModeMacro = middleAutonModeMacro;
                break;
            case BLUE_LEFT:
                selectedModeMacro = blueLeftAutonModeMacro;
                break;
            case BLUE_RIGHT:
                selectedModeMacro = blueRightAutonModeMacro;
                break;
            case RED_LEFT:
                selectedModeMacro = redLeftAutonModeMacro;
                break;
            case RED_RIGHT:
                selectedModeMacro = redRightAutonModeMacro;
                break;
            case ROTATE_LEFT_360:
                selectedModeMacro = rotateLeftStateMacro;
                break;
            case ROTATE_LEFT_FULL:
            	selectedModeMacro = rotateFullLeftStateMacro;
            	break;
            case ROTATE_RIGHT_360:
                selectedModeMacro = rotateRightStateMacro;
                break;
            case FORWARD_6:
                selectedModeMacro = forwardStateMacro;
                break;
            case FORWARD_TEST:
            	selectedModeMacro = forwardEmpericalMacro;
            	break;
            case BACKWARD_6:
                selectedModeMacro = backwardStateMacro;
                break;
            // AZ STEP TWO: COPY PASTE AND REPLACE NAMES
            case DEMO_NEW_AUTON:
                selectedModeMacro = new DemoStateMacro();
                break;
            /*case SHOOTER_DEMO:
            	selectedModeMacro = new ShooterDemoMacro();
            	break;*/
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
    
    /*protected final class ShooterCoordinator extends Coordinator {
    	private SmartTimer timeElapsed = new SmartTimer();
    	private Shooter.ShooterStartup shooterStartup = robot.shooter.new ShooterStartup();
    	private Loader.Load load = robot.loader.new Load();
    	
    	private double time;
    	
    	public ShooterCoordinator() {
    		this(0);
    	}
    	
    	public ShooterCoordinator(double time) {
    		super();
    		this.time = time;
    	}
    	
    	@Override
    	protected void begin() {
    		timeElapsed.start();
    	}
    	
    	@Override
    	protected boolean run() {
    		if( timeElapsed.get() >= Calibration.MIN_CHARGE_TIME 
    				&& timeElapsed.get() < time + Calibration.MIN_CHARGE_TIME ) {
    			load.on.set(true);
    			load.activate();
    		}
    		if( timeElapsed.get() < time + Calibration.MIN_CHARGE_TIME ) {
    			shooterStartup.motorSpeed.set(Calibration.AUTON_SHOOTER_INPUT);
    			shooterStartup.maximumOverdrive.set(false);
    			shooterStartup.activate();
    			return true;
    		} else {
    			return false;
    		}
    	}
    	
    	@Override
    	protected void end() {
    		shooterStartup.motorSpeed.set(0D);
    		load.on.set(false);
    		timeElapsed.stopAndReset();
    	}
    }*/
    
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
		        return robot.drive.rightClicks.get()-robot.drive.leftClicks.get();
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
			return moveController.getAvgError();
		}
		
		public double getRotError() {
			return rotController.getAvgError();
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
		    System.out.println("Move error is " + getMoveError() + ", Rot error is " + getRotError());
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
		    arcadePIDLog.log("INFO","Final Rotate error is "+rotController.getAvgError());
		    arcadePIDLog.log("INFO","Final Move error is "+moveController.getAvgError());
		    rotController.disable();
		    moveController.disable();
		    timeElapsed.stopAndReset();
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
        
        public double getMoveError() {
        	return controller.getMoveError();
        }
        public double getRotError() {
        	return controller.getRotError();
        }
    }
    
    private class DemoStateMacro extends StatefulCoordinator {
    	public DemoStateMacro() {
    		super(DemoStateMacro.class);
    		addState("Forward 12 feet", new ArcadePIDCoordinator(AutonMovement.empericalInchesToClicks(Calibration.DRIVE_PROPERTIES, 12*12), 0));
    		addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
    		addState("Rotate 180 right", new ArcadePIDCoordinator(0,AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 180)));
    		addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
    		addState("Forward 12 feet", new ArcadePIDCoordinator(AutonMovement.empericalInchesToClicks(Calibration.DRIVE_PROPERTIES, 12*12), 0));
    	}
    }

    /*private class ShooterDemoMacro extends StatefulCoordinator {
    	public ShooterDemoMacro () {
    		super(ShooterDemoMacro.class);
    		
    		addState("Shooter", new ShooterCoordinator(Calibration.AUTON_SHOOTER_TIME));
    	}
    }*/
    
    // AZ STEP ONE: CREATE METHOD
    /*
    private class AZAutonModeMiddle extends StatefulCoordinator {
    // Remname to fit whatever, fix constructors class name
    	public AZAutonModeMiddle() {
    		super(AZAutonModeMiddle.class);
    		// USE THESE 4
    		// COPY PASTE
    		// inches
    		// degrees
    		addState("Forward 12 feet", new ArcadePIDCoordinator(AutonMovement.empericalInchesToClicks(Calibration.DRIVE_PROPERTIES, 12*12), 0));
    		addState("Sleep 0.5 seconds", new SleepCoordinator(0.5));
    		addState("Rotate 180 right", new ArcadePIDCoordinator(0,AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 180)));
    		addState("Shooter", new ShooterCoordinator(Calibration.AUTON_SHOOTER_TIME));

    	}
    }
    */
    
    private class MiddleAutonModeMacro extends StatefulCoordinator {
        public MiddleAutonModeMacro () {
            super(MiddleAutonModeMacro.class);

//            addState("driveForward", new TimedDriveMacro(robot) {
//                @Override
//                protected double getLeftPower () { return robot.dashboard.middleAutonDriveForwardLeftPower.get(); }
//                @Override
//                protected double getRightPower () { return robot.dashboard.middleAutonDriveForwardRightPower.get(); }
//                @Override
//                protected double getTime () { return robot.dashboard.middleAutonDriveForwardTime.get(); }
//            });
            // Continues forward to cross the baseline without crossing midline
            addState("driveForward", new ArcadePIDCoordinator(AutonMovement.inchesToClicks(Calibration.DRIVE_PROPERTIES, 85),// was 12*8-3
            		AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 2.5)));
        }
    }

    private abstract class SideAutonModeMacro extends StatefulCoordinator {
        public SideAutonModeMacro (String name) {
            super(name);
            addState("Forward", new ArcadePIDCoordinator(AutonMovement.empericalInchesToClicks(Calibration.DRIVE_PROPERTIES, 12*12), 0));
    		addState("Sleep 0.25 seconds", new SleepCoordinator(0.25));
    		addState("Rotate Left", new ArcadePIDCoordinator(0,AutonMovement.degreesToClicks(Calibration.DRIVE_PROPERTIES, 60)));
    		addState("Sleep 0.25 seconds again", new SleepCoordinator(0.25));
    		addState("Forward Again", new ArcadePIDCoordinator(AutonMovement.empericalInchesToClicks(Calibration.DRIVE_PROPERTIES, 12*12), 0));

            
        }

        protected abstract double getDriveForwardClicks ();
        protected abstract double getTurnToFacePegAngle ();
    }
}