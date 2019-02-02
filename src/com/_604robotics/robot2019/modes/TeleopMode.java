package com._604robotics.robot2019.modes;

import com._604robotics.marionette.InputPlayer;
import com._604robotics.marionette.InputRecorder;
import com._604robotics.marionette.InputRecording;
import com._604robotics.marionette.MarionetteJoystick;
import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.modules.Drive;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.controller.ExtendablePIDController;
import com._604robotics.robotnik.prefabs.flow.Pulse;
import com._604robotics.robotnik.prefabs.flow.Toggle;
import com._604robotics.robotnik.prefabs.inputcontroller.xbox.XboxController;
import com._604robotics.robotnik.prefabs.modules.Limelight;
import edu.wpi.first.wpilibj.PIDOutput;

import java.io.IOException;

public class TeleopMode extends Coordinator {

    private static final Logger logger = new Logger(TeleopMode.class);

    private final InputPlayer inputPlayer = new InputPlayer();
    private InputRecorder inputRecorder;

    private final MarionetteJoystick driverJoystick = new MarionetteJoystick(0, inputPlayer, 0);
    private final MarionetteJoystick manipJoystick = new MarionetteJoystick(1, inputPlayer, 1);

    private final XboxController driver = new XboxController(driverJoystick);
    private final XboxController manip = new XboxController(manipJoystick);

    private final com._604robotics.robot2019.Robot2019 robot;

    private final DriveManager driveManager;
    private final AutoCenterManager autoCenterManager;
    private final Logger test = new Logger("Teleop");

    public TeleopMode ( com._604robotics.robot2019.Robot2019 robot) {
        driver.leftStick.x.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);
        driver.leftStick.y.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);

        driver.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
        driver.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

        driver.rightStick.x.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);
        driver.rightStick.y.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);

        //driver.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
        driver.rightStick.x.setFactor(1); // WEIRD_WHY_?FES:RLJTH *ROHT guirg
        driver.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);

        manip.leftStick.x.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);
        manip.leftStick.y.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);

        manip.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
        manip.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

        manip.rightStick.x.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);
        manip.rightStick.y.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);

        manip.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
        manip.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);

        this.robot = robot;

        driveManager = new DriveManager();
        autoCenterManager = new AutoCenterManager();
    }

    private boolean getHoldArmClicks = false;

    private double driverLeftJoystickY = 0.0;
    private double driverLeftJoystickX = 0.0;
    private double driverLeftTrigger = 0.0;

    private boolean driverLeftJoystickButton = false;
    private boolean driverLeftTriggerButton = false;
    private boolean driverLeftBumper = false;

    private double driverRightJoystickY = 0.0;
    private double driverRightJoystickX = 0.0;
    private double driverRightTrigger = 0.0;

    private boolean driverRightJoystickButton = false;
    private boolean driverRightTriggerButton = false;
    private boolean driverRightBumper = false;

    private boolean driverBack = false;
    private boolean driverStart = false;
    private boolean driverA = false;
    private boolean driverB = false;
    private boolean driverX = false;
    private boolean driverY = false;

    private boolean driverDPad = false;

    private double manipLeftJoystickY = 0.0;
    private double manipLeftJoystickX = 0.0;
    private double manipLeftTrigger = 0.0;

    private boolean manipLeftJoystickButton = false;
    private boolean manipLeftTriggerButton = false;
    private boolean manipLeftBumper = false;

    private double manipRightJoystickY = 0.0;
    private double manipRightJoystickX = 0.0;
    private double manipRightTrigger = 0.0;

    private boolean manipRightJoystickButton= false;
    private boolean manipRightTriggerButton= false;
    private boolean manipRightBumper= false;

    private boolean manipBack= false;
    private boolean manipStart= false;
    private boolean manipA= false;
    private boolean manipB= false;
    private boolean manipX= false;
    private boolean manipY= false;
    private boolean manipDPad = false;

    public void startPlayback (InputRecording recording) {
        inputPlayer.startPlayback(recording);
    }

    public void stopPlayback () {
        inputPlayer.stopPlayback();
    }

    @Override
    protected void begin () {
        if (inputPlayer.isPlaying()) {
            logger.info("Playing back Marionette recording");
        } else if (robot.dashboard.recordAuton.get()) {
            logger.info("Recording inputs with Marionette");
            inputRecorder = new InputRecorder(2400, driverJoystick, manipJoystick);
        }
    }

    @Override
    protected boolean run () {
        updateControls();
        process();
        return true;
    }

    @Override
    protected void end () {
        if (inputRecorder != null) {
            final InputRecorder oldInputRecorder = inputRecorder;
            inputRecorder = null;

            try {
                logger.info("Terminating Marionette recording");
                oldInputRecorder.close();

                // filename is prefixed when filename is saved to
                String fileName = robot.dashboard.marionetteFile.get();
                switch( robot.dashboard.marionetteRecorder.get() ) {
                    case MANUAL:
                        if( Calibration.AUTO_APPEND_TIMESTAMP ) {
                            fileName = System.currentTimeMillis() + "_" + fileName;
                        }
                        break;
                    default:
                        break;
                }
                logger.info("Saving Marionette recording to \"" + robot.dashboard.filePrefix.get() + fileName + "\"");
                oldInputRecorder.getRecording().save("/home/lvuser/" + robot.dashboard.filePrefix.get() + fileName);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private void updateControls() {
        driverLeftJoystickY = driver.leftStick.y.get();
        driverLeftJoystickX = driver.leftStick.x.get();
        driverLeftTrigger = driver.triggers.left.get();

        driverLeftJoystickButton = driver.buttons.leftStick.get();
        driverLeftTriggerButton = driver.buttons.lt.get();
        driverLeftBumper = driver.buttons.lb.get();

        driverRightJoystickY = driver.rightStick.y.get();
        driverRightJoystickX = driver.rightStick.x.get();
        driverRightTrigger = driver.triggers.right.get();

        driverRightJoystickButton = driver.buttons.rightStick.get();
        driverRightTriggerButton = driver.buttons.rt.get();
        driverRightBumper = driver.buttons.rb.get();

        driverBack = driver.buttons.back.get();
        driverStart = driver.buttons.start.get();
        driverA = driver.buttons.a.get();
        driverB = driver.buttons.b.get();
        driverX = driver.buttons.x.get();
        driverY = driver.buttons.y.get();

        driverDPad = driver.dpad.pressed.get();

        manipLeftJoystickY = manip.leftStick.y.get();
        manipLeftJoystickX = manip.leftStick.x.get();
        manipLeftTrigger = manip.triggers.left.get();

        manipLeftJoystickButton = manip.buttons.leftStick.get();
        manipLeftTriggerButton = manip.buttons.lt.get();
        manipLeftBumper = manip.buttons.lb.get();

        manipRightJoystickY = manip.rightStick.y.get();
        manipRightJoystickX = manip.rightStick.x.get();
        manipRightTrigger = manip.triggers.right.get();

        manipRightJoystickButton = manip.buttons.rightStick.get();
        manipRightTriggerButton = manip.buttons.rt.get();
        manipRightBumper = manip.buttons.rb.get();

        manipBack = manip.buttons.back.get();
        manipStart = manip.buttons.start.get();
        manipA = manip.buttons.a.get();
        manipB = manip.buttons.b.get();
        manipX = manip.buttons.x.get();
        manipY = manip.buttons.y.get();

        manipDPad = manip.dpad.pressed.get();
    }

    private void process() {
        driveManager.run();
    }

    private class DriveManager {
        private final Drive.ArcadeDrive arcade;
        private final Drive.TankDrive tank;
        private final Drive.Idle idle;
        private CurrentDrive currentDrive;
        private Toggle inverted;

        public DriveManager () {
            idle = robot.drive.new Idle();
            arcade = robot.drive.new ArcadeDrive();
            tank = robot.drive.new TankDrive();
            // TODO: Expose on dashboard
            currentDrive = CurrentDrive.ARCADE;
            // TODO: Expose on dashboard
            inverted = new Toggle(false);
        }

        public void run() {
            double leftY = driver.leftStick.y.get();
            double rightY = driver.rightStick.y.get();
            double rightX = driver.rightStick.x.get();

            // Flip values if xbox inverted
            inverted.update(driverLeftBumper);
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

            if( driverX ) {
                // Activate Limelight detection
				robot.limelight.scan.activate();
				//robot.dashboard.limelightVisionMode.set(LimelightMode.VISION);
                arcade.movePower.set(leftY); // Allow driver to control dist from target
                arcade.activate();
                autoCenterManager.run();
				System.out.println("AUTOCENTER");
            } else {
                autoCenterManager.end();
				
				switch(robot.dashboard.limelightVisionMode.get()) {
					case DRIVER:
						robot.limelight.driver.activate();
						break;
					case VISION:
						robot.limelight.scan.activate();
						break;
					default:
						robot.limelight.scan.activate();
				
                } switch( currentDrive ) {
                    case IDLE:
                        idle.activate();
                        break;
                    case ARCADE:
                        arcade.movePower.set(leftY);
                        if( driverLeftJoystickButton ) {
                            arcade.rotatePower.set(rightX * Calibration.SLOW_ROTATION_MODIFIER);
                        } else {
                            arcade.rotatePower.set(rightX);
                        }
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
    }

    private class AutoCenterManager {
        private ExtendablePIDController anglePID;
        private ExtendablePIDController distPID;
        private PIDOutput rotation;
        private PIDOutput drive;

        public AutoCenterManager() {
            rotation = new PIDOutput() {
                @Override
                public synchronized void pidWrite(double output) {
                    driveManager.arcade.rotatePower.set(output);
                }
            };
            drive = new PIDOutput() {
                @Override
                public synchronized void pidWrite(double output) {
                    driveManager.arcade.movePower.set(output);
                }
            };
            anglePID = new ExtendablePIDController(-0.02, 0, -0.15, new Limelight.HorizontalError(robot.limelight,0), rotation);
            anglePID.setAbsoluteTolerance(Calibration.LIMELIGHT_ANGLE_TOLERANCE);
            distPID = new ExtendablePIDController(0.5, 0, 0, new Limelight.DistanceError(robot.limelight, 18), drive);
            distPID.setAbsoluteTolerance(Calibration.LIMELIGHT_DIST_TOLERANCE);
        }

        public void run() {
			
            //robot.limelight.scan.activate();
			System.out.println("RUNNNNNNNNNNNNNNNNNNNNNNN");
            if( robot.limelight.limelightHasTargets.get() ) {
				System.out.println("ANGLERPIDDDDD");
                anglePID.setEnabled(true);
            } else {
                this.end();
            }
        }

        public void end() {
            anglePID.setEnabled(false);
            anglePID.reset();
            distPID.setEnabled(false);
            //robot.limelight.driver.activate();
        }
    }

    private enum CurrentDrive {
        IDLE, ARCADE, TANK
    }
}
