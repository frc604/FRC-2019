package com._604robotics.robot2019.modes;

import com._604robotics.robot2019.Robot2017;
import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.modules.Climber;
import com._604robotics.robot2019.modules.Drive;
import com._604robotics.robot2019.modules.Loader;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import com._604robotics.robotnik.prefabs.flow.Toggle;
import com._604robotics.robotnik.prefabs.inputcontroller.xbox.XboxController;

public class TeleopMode extends Coordinator {
	private boolean started = false;
	
    private final XboxController driver = new XboxController(0);
    private final XboxController manip = new XboxController(1);

    private final Robot2017 robot;

    private final ClimberManager climberManager;
    private final SignalLightManager signalLightManager;
    private final PickupManager pickupManager;
    private final DriveManager driveManager;
    //private final ShooterManager shooterManager;
    private final LoaderManager loaderManager;
    
    
    
    public TeleopMode (Robot2017 robot) {
        driver.leftStick.x.setDeadband(Calibration.TELEOP_DEADBAND);
        driver.leftStick.y.setDeadband(Calibration.TELEOP_DEADBAND);

        driver.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
        driver.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

        driver.rightStick.x.setDeadband(Calibration.TELEOP_DEADBAND);
        driver.rightStick.y.setDeadband(Calibration.TELEOP_DEADBAND);

        driver.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
        driver.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);

        manip.leftStick.x.setDeadband(Calibration.TELEOP_DEADBAND);
        manip.leftStick.y.setDeadband(Calibration.TELEOP_DEADBAND);

        manip.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
        manip.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

        manip.rightStick.x.setDeadband(Calibration.TELEOP_DEADBAND);
        manip.rightStick.y.setDeadband(Calibration.TELEOP_DEADBAND);

        manip.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
        manip.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);
        
        this.robot = robot;

        climberManager = new ClimberManager();
        signalLightManager = new SignalLightManager();
        pickupManager = new PickupManager();
        driveManager = new DriveManager();
        //shooterManager = new ShooterManager();
        loaderManager = new LoaderManager();
    }

    @Override
    public boolean run () {
        climberManager.run();
        signalLightManager.run();
        pickupManager.run();
        driveManager.run();
        //shooterManager.run();
        loaderManager.run();
        return true;
    }

    private class ClimberManager {
        private final Climber.Climb climb;

        public ClimberManager () {
            climb = robot.climber.new Climb();
        }

        public void run () {
            /*if (driver.buttons.lt.get()) {
                climb.power.set(driver.triggers.left.get());
                climb.activate();
            }*/
        	if (driver.buttons.start.get()) {
        		climb.power.set(1.0);
        		climb.activate();
        	} else if (driver.buttons.back.get()) {
        		climb.power.set(-1.0);
        		climb.activate();
        	}
        }
    }


    /*public class ShooterManager {
    	private final Shooter.Idle shooterIdle;
        private final Shooter.ShooterStartup shooterStartup;
        
        public ShooterManager() {
        	shooterIdle = robot.shooter.new Idle();
            shooterStartup = robot.shooter.new ShooterStartup();
        }
        
        public void run() {
        	double manipLT = manip.triggers.left.get();      
        	boolean manipLB = manip.buttons.lb.get();
        	if( manipLT > 0 ) {
                shooterStartup.motorSpeed.set(manipLT);
                shooterStartup.maximumOverdrive.set(manipLB);
                shooterStartup.activate();
        	} else {
        		shooterIdle.activate();
        	}
        }
    }*/
    
    public class LoaderManager {
    	private final Loader.Idle loaderIdle;
        private final Loader.Load load;
        
        public LoaderManager() {
        	loaderIdle = robot.loader.new Idle();
            load = robot.loader.new Load();
        }
        
        public void run() {
            boolean manipRB = manip.buttons.rb.get();

            if( manipRB )
            {
            	load.on.set(manipRB);
            	load.activate();
            } else {
            	loaderIdle.activate();
            }
        }
    }

    private enum CurrentDrive {
        IDLE, ARCADE, TANK
    }

    private class DriveManager {
        private final Drive.ArcadeDrive arcade;
        private final Drive.TankDrive tank;
        private final Drive.Idle idle;
        private CurrentDrive currentDrive;
        private Toggle inverted;
		private Toggle gearState;

        public DriveManager () {
            idle=robot.drive.new Idle();
            arcade=robot.drive.new ArcadeDrive();
            tank=robot.drive.new TankDrive();
            // TODO: Expose on dashboard
            currentDrive=CurrentDrive.ARCADE;
            // TODO: Expose on dashboard
            inverted=new Toggle(false);
            gearState=new Toggle(false);
        }

        public void run() {
        	// Set gears
        	gearState.update(driver.buttons.lb.get());
        	if (gearState.isInOnState()) {
        		robot.shifter.highGear.activate();
        	} else if (gearState.isInOffState()) {
        		robot.shifter.lowGear.activate();
        	}
        	// Get Xbox data
            double leftY=driver.leftStick.y.get();
            double rightX=driver.rightStick.x.get();
            double rightY=driver.rightStick.y.get();
            // Flip values if xbox inverted
            inverted.update(driver.buttons.rb.get());
            robot.dashboard.XboxFlipped.set(inverted.isInOnState());
            if (inverted.isInOnState()) {
                leftY*=-1;
                rightY*=-1;
            }
            // Get Dashboard option for drive
            switch (robot.dashboard.driveMode.get()){
                case OFF:
                    currentDrive=CurrentDrive.IDLE;
                    break;
                case ARCADE:
                    currentDrive=CurrentDrive.ARCADE;
                    break;
                case TANK:
                    currentDrive=CurrentDrive.TANK;
                    break;
                case DYNAMIC:
                    // Dynamic Drive mode detection logic
                    if (currentDrive == CurrentDrive.TANK) {
                        if (Math.abs(rightY) <= 0.2 && Math.abs(rightX) > 0.3) {
                            currentDrive = CurrentDrive.ARCADE;
                        }
                    } else { // currentDrive == CurrentDrive.ARCADE
                        if (Math.abs(rightX) <= 0.2 && Math.abs(rightY) > 0.3) {
                            currentDrive = CurrentDrive.TANK;
                        }
                    }
                    break;
                default:
                    System.out.println("This should never happen!");
                    System.out.println("Current value is:"+robot.dashboard.driveMode.get());
            }

            // Set appropriate drive mode depending on dashboard option
            switch (currentDrive) {
                case IDLE:
                    idle.activate();
                    break;
                case ARCADE:
                    arcade.movePower.set(leftY);
                    arcade.rotatePower.set(rightX);
                    arcade.activate();
                    break;
                case TANK:
                    tank.leftPower.set(leftY);
                    tank.rightPower.set(rightY);
                    tank.activate();
                    break;
            }

        }
    }

    private class SignalLightManager {
        private final Toggle lightToggle = new Toggle(false);

        public void run () {
            lightToggle.update(driver.buttons.rt.get());
            if (lightToggle.isInOnState()) {
                robot.signalLight.on.activate();
            }
        }
    }

    private enum IntakeState {
        IDLE, FORWARD_SPIT, REVERSE_SUCK
    }

    private class PickupManager {
        private boolean extend = false;
        private IntakeState intakeState = IntakeState.IDLE;
        private SmartTimer retractSpinTimer = new SmartTimer();

        public void run () {
            if (driver.buttons.y.get()) {
                extend = false;
            }
            if (driver.buttons.x.get() || driver.buttons.b.get() || driver.buttons.a.get()) {
                extend = true;
            }

            if (driver.buttons.y.get()) {
                intakeState = IntakeState.IDLE;
            }
            if (driver.buttons.x.get()) {
                intakeState = IntakeState.REVERSE_SUCK;
            }
            if (driver.buttons.b.get()) {
                intakeState = IntakeState.FORWARD_SPIT;
            }
            if( !retractSpinTimer.isRunning() && !started ) {
            	retractSpinTimer.start();
            } else if( retractSpinTimer.hasPeriodPassed(1D) && !started ) {
            	started = true;
            }
            if (extend) {
                robot.flipFlop.extend.activate();
                retractSpinTimer.stopAndReset();
            } else {
                robot.flipFlop.retract.activate();
                intakeState = IntakeState.IDLE;
                
                if (!retractSpinTimer.isRunning() && started) {
                	retractSpinTimer.start();
                	intakeState = IntakeState.REVERSE_SUCK;
                } else if( retractSpinTimer.hasPeriodPassed(1D) ) {
                	intakeState = IntakeState.IDLE;
                }
            }

            switch (intakeState) {
                case FORWARD_SPIT:
                    robot.intake.spit.activate();
                    break;
                case REVERSE_SUCK:
                    robot.intake.suck.activate();
                    break;
                case IDLE:
                    robot.intake.idle.activate();
                    break;
            }
        }
    }
}
