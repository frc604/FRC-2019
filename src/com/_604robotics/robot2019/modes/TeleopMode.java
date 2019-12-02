package com._604robotics.robot2019.modes;

import com._604robotics.marionette.InputPlayer;
import com._604robotics.marionette.InputRecorder;
import com._604robotics.marionette.InputRecording;
import com._604robotics.marionette.MarionetteJoystick;
import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.modules.Drive;
import com._604robotics.robot2019.modules.Intake;
import com._604robotics.robot2019.modules.ProfiledArm;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.controller.NewExtendablePIDController;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import com._604robotics.robotnik.prefabs.flow.Toggle;
import com._604robotics.robotnik.prefabs.inputcontroller.xbox.XboxController;
import com._604robotics.robotnik.prefabs.modules.Limelight;

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
    private final ArmManager armManager;
    private final IntakeManager intakeManager;
    private final HatchManager hatchManager;
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
        armManager = new ArmManager();
        intakeManager = new IntakeManager();
        hatchManager = new HatchManager();
        autoCenterManager = new AutoCenterManager();
    }

    //<editor-fold desc="Getting Controller Values"
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

    private boolean hatchCollisionChecker;
    private boolean armCollisionChecker;
    //</editor-fold>

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

        robot.intake.speed.set(0);
        hatchCollisionChecker = false;
        armCollisionChecker = false;

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
        armManager.run();
        intakeManager.run();
        hatchManager.run();
    }

    private class DriveManager {
        private final Drive.ArcadeDrive arcade;
        private final Drive.TankDrive tank;
        private final Drive.Idle idle;
        private CurrentDrive currentDrive;
        private CurrentDrive selectedDrive;
        private Toggle inverted;

        public DriveManager () {
            idle = robot.drive.new Idle();
            arcade = robot.drive.new ArcadeDrive();
            tank = robot.drive.new TankDrive();
            currentDrive = CurrentDrive.ARCADE;
            inverted = new Toggle(false);
        }

        public void run() {
            robot.drive.updateOdometry();
            double leftY = driver.leftStick.y.get();
            double rightY = driver.rightStick.y.get();
            double rightX = driver.rightStick.x.get();

            if (driverLeftJoystickButton) {
                leftY *= 0.8;
                rightY *= 0.8;
                rightX *= 0.8;
            }

            inverted.update(driverLeftBumper);
            robot.dashboard.XboxFlipped.set(inverted.isInOnState());
            if (inverted.isInOnState()) { // Flip values if xbox inverted
                leftY *= -1;
                rightY *= -1;
            }

            // Get Dashboard option for drive
            switch (robot.dashboard.driveMode.get()){
                case OFF:
                    currentDrive=CurrentDrive.IDLE;
                    selectedDrive=CurrentDrive.IDLE;
                    break;
                case ARCADE:
                    currentDrive=CurrentDrive.ARCADE;
                    selectedDrive=CurrentDrive.ARCADE;
                    break;
                case TANK:
                    currentDrive=CurrentDrive.TANK;
                    selectedDrive=CurrentDrive.TANK;
                    break;
                case DYNAMIC:
                    // Dynamic Drive mode detection logic
                    if (currentDrive == CurrentDrive.TANK) {
                        if (Math.abs(rightY) <= 0.2 && Math.abs(rightX) > 0.3) {
                            currentDrive = CurrentDrive.ARCADE;
                            selectedDrive=CurrentDrive.ARCADE;
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

            //Climber Code
            if ( driverStart ) {

                //Preperation for climb
                robot.powermonitor.updateCompressor(false);

                if ( robot.slider.isForward.get() ) {
                    hatchManager.sliderForward.update(true); //Check if slider is forward, if true update the toggle to move it back.
                }

                armManager.disableArm = true; //Disabling arm control from Arm Manager

                if ( robot.arm.leftEncoderClicks.get() >= (Calibration.Arm.LOW_SETPOINT - Calibration.Arm.LOW_SETPOINT * 0.25) ) {  
                    robot.arm.move.inputPower.set(-0.10); // Arm downwards power, NOTE: needs to overcome gravity
                    robot.arm.move.activate();
                } else {
                    robot.arm.setpoint.setpoint.set(Calibration.Arm.LOW_SETPOINT);//If arm has not reach low setpont keep activating the setpoint till it does.
                    robot.arm.setpoint.activate();
                }

                // Start Climb
                if ( driverDPad ) {
                    robot.tilter.tilt.activate(); // Climbs at 100% power
                    currentDrive = CurrentDrive.ARCADE; 
                    arcade.activate();  
                    arcade.movePower.set(-0.5); //Drive backwards power
                    arcade.activate();  
                } else if ( driverBack ) {
                    robot.tilter.retract.activate(); //Retracts at 30% power
                } else {
                    robot.tilter.stow.activate();
                }

                currentDrive = CurrentDrive.MANUAL; //Disable driver control

            } else {
                armManager.disableArm = false;
                robot.powermonitor.updateCompressor(true);
                robot.tilter.stow.activate();
                currentDrive = selectedDrive;
            }

            //Limelight Activation Code
            // It would be bad if this turned off
            robot.limelight.limelightLED.set(Limelight.LEDState.ON.ordinal());
            if( driverRightBumper ) {
                robot.limelight.scan.activate();
                arcade.activate();

                if( robot.limelight.limelightHasTargets.get() ) {
                    currentDrive = CurrentDrive.MANUAL; //Disable manual control so the PID can take over
                    arcade.movePower.set(leftY); //Still allow driver to control forward/backwards movement
                    autoCenterManager.run();
                    if ( autoCenterManager.anglePID.atSetpoint() ) {
                        driver.rumble.setEnabled(true);
                        driver.rumble.setRumble(1);
                    } else {
                        driver.rumble.setEnabled(false);
                    }
                } else {
                    driver.rumble.setEnabled(false);
                    robot.limelight.scan.activate();
                    currentDrive = selectedDrive;
                }
                
            } else {
                autoCenterManager.end();

                switch(robot.dashboard.limelightVisionMode.get()){
                    case DRIVER:
                        robot.limelight.driver.activate();
                        break;
                    case VISION:
                        robot.limelight.scan.activate();
                        break;
                    default:
                        robot.limelight.scan.activate();
                }
            }

            switch( currentDrive ) {
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
                case MANUAL: //Disable driver control of the robot
                    break;
            }
        }
    }


    private class IntakeManager {
        private final Intake.Idle idle;
        private final Intake.Speed speed;

        public IntakeManager() {
            idle = robot.intake.idle;
            speed = robot.intake.speed;
        }

        public void run() {
            if( driverRightTrigger != 0.0 ) {
                speed.set(-driverRightTrigger); // Intake
                //Negative is Intake

            } else if( driverLeftTrigger != 0.0 ){
                speed.set( Math.min(driverLeftTrigger, 0.80) ); // Outtake
                //Clamping output to reduce outake speed

            } else if( manipLeftTrigger != 0.0 ) {
                speed.set(-manipLeftTrigger ); // Intake

            } else if( manipRightTrigger != 0.0 ) {
                speed.set( Math.min(manipRightTrigger, 0.75) ); // Outtake
                //Clamping output to reduce outake speed

            } else if( manipDPad ) {
                speed.set(1); //Force spit

            } else {
                idle.activate(); 
            }
        }
    }

    private class ArmManager {
        private ProfiledArm arm;
        private Toggle hardstopToggle;
        protected boolean disableArm;
        private Toggle manualHardstop;

        public ArmManager() {
            arm = robot.arm;
            disableArm = false;
            manualHardstop = new Toggle(false);
            hardstopToggle = new Toggle(false);
        }

        public void run() {
            if( arm.setpoint.setpoint.get() == null ) {
                // Is never set to null, yet spazzes out anyways
                // Perhaps setting to non-persistent?
                // Setpoint is null when hold action is running
                arm.setpoint.setpoint.set(0.0);
            }

            if ( arm.leftEncoderClicks.get() >= 1300 ){
                armCollisionChecker = true;
            } else {
                armCollisionChecker = false;
            }

            if ( manipBack ) {
                arm.resetEncoder();
            }

            if ( !disableArm ) {

                if ( hardstopToggle.isInOffState() ) {
                    robot.hardstop.close.activate();
                } else if ( hardstopToggle.isInOnState() ) {
                    robot.hardstop.open.activate();
                }

                /*Hardstop Coordination code*/
                if ( manipLeftBumper ) {
                    hardstopToggle.update(manipLeftBumper);
                    manualHardstop.update(manipLeftBumper);
                } else if ( manualHardstop.isInOffState() &&
                        ( arm.leftEncoderClicks.get() >= Calibration.Arm.HARDSTOP_CLOSE_POSITION ) &&
                        ( (arm.hold.isRunning() ? 300.0 : arm.setpoint.setpoint.get()) >= Calibration.Arm.HARDSTOP_CLOSE_POSITION ) ) {
                    hardstopToggle.update(false);
                    hardstopToggle.update(hardstopToggle.isInOnState());
                } else if ( manualHardstop.isInOffState() ) {
                    hardstopToggle.update(false);
                    hardstopToggle.update(hardstopToggle.isInOffState());
                }
                

                // Check setpoints
                if( manipA && !hatchCollisionChecker ) {
                    // Low position // ARMSETPOINTS
                    arm.setpoint.setpoint.set(Calibration.Arm.LOW_SETPOINT);
                    arm.setpoint.activate();
                } else if( manipB ) {
                    // Ball place position
                    arm.setpoint.setpoint.set(Calibration.Arm.OUTPUT_SETPOINT);
                    arm.setpoint.activate();
                } else if( manipY ) {
                    // Back Scoring Rocket
                    arm.setpoint.setpoint.set(Calibration.Arm.BACK_ROCKET_SETPOINT);
                    arm.setpoint.activate();
                } else if( manipX && !hatchCollisionChecker ) {
                    // Vertical position
                    arm.setpoint.setpoint.set(Calibration.Arm.ROCKET_SETPOINT);
                    arm.setpoint.activate();
                } else if( manipRightBumper ) {
                    //Back Scoring Cargo
                    arm.setpoint.setpoint.set(Calibration.Arm.BACK_CARGO_SETPOINT);
                    arm.setpoint.activate();
                } else {
                    // Check thumbsticks
                    if( manipLeftJoystickY != 0 ) {
                        // Set arm rate to joystick
                        double motorValue = manipLeftJoystickY * Calibration.Arm.SCALE_JOYSTICK;

                        // Calculate needed factor for torque
                        double angle = 2*Math.PI * (arm.leftEncoderClicks.get() - Calibration.Arm.HORIZONTAL_POSITION) / Calibration.Arm.CLICKS_FULL_ROTATION;
                        angle = Math.cos(angle);

                        if( (motorValue < 0 && arm.leftEncoderClicks.get() < Calibration.Arm.VERTICAL_POSITION) ||
                            (motorValue > 0 && arm.leftEncoderClicks.get() > Calibration.Arm.VERTICAL_POSITION) ) {

                            // We need to account for gravity existing
                            motorValue += Calibration.Arm.kF * angle;
                        }

                        arm.move.inputPower.set(motorValue);
                        arm.move.activate();
                    } else {
                        // Hold arm still
                        arm.hold.activate();
                    }

                }

            }

        }

    }

    private class HatchManager {
        private Toggle hookToggle;
        protected Toggle sliderForward;
        private SmartTimer hatchTime;
        private int autoState = 0;
        private int intakeState = 0;

        public HatchManager() {
            hookToggle = new Toggle(false); // Assuming the piston is in the held state to start
            sliderForward = new Toggle(false); // Not extended initially
            hatchTime = new SmartTimer(); 
            //TODO: Test if you can have multiple timers per thread
        }

        public void run() {

            if ( robot.slider.isForward.get() ){
                hatchCollisionChecker = true;
            } else {
                hatchCollisionChecker = false;
            }

            if( driverA ) {
                hookToggle.update(driverA);
            } else {
                hookToggle.update(false);
            }

            if ( hookToggle.isInOffState() ) {
                robot.hook.hold.activate();
            } else if ( hookToggle.isInOnState() ) {
                robot.hook.release.activate();
            }

            if( driverY ) {
                sliderForward.update( driverY );
            } else {
                sliderForward.update(false);
            }

            if ( !armCollisionChecker ){
                if( sliderForward.isInOnState() ) {
                    robot.slider.front.activate();
                } else if( sliderForward.isInOffState() ) {
                    robot.slider.back.activate();
                }
            }

            switch (autoState) {
                case( 0 ):
                    if ( driverB || manipRightJoystickY != 0.0 ){
                        autoState++;
                    }
                    break;

                case ( 1 ):
                    sliderForward.update(sliderForward.isInOffState());
                    hatchTime.startIfNotRunning();
                    autoState++;
                    break;

                case ( 2 ):
                    if( hatchTime.hasPeriodPassed(Calibration.HATCH_PUSH_TIME) ) {
                        hookToggle.update(!hookToggle.isInOnState());
                        hatchTime.stopAndReset();
                        hatchTime.startIfNotRunning();
                        autoState++;
                    }
                    break;

                case( 3 ):
                    if( hatchTime.hasPeriodPassed(Calibration.HATCH_BACK_TIME) ) {
                        sliderForward.update(sliderForward.isInOnState());
                        hatchTime.stopAndReset();
                        driver.rumble.setEnabled(false);
                        autoState++;

                    } else {
                        driver.rumble.setEnabled(true);
                        driver.rumble.setRumble(1);
                    }
                    break;
                case ( 4 ):
                    autoState = 0;
                    break;
            }

            if ( autoState == 0 ) {
                switch (intakeState) {
                    case( 0 ):
                        if ( driverX ){
                            intakeState++;
                        }
                        break;

                    case ( 1 ):
                        sliderForward.update(sliderForward.isInOffState());
                        hatchTime.startIfNotRunning();
                        intakeState++;
                        break;

                    case ( 2 ):
                        if( hatchTime.hasPeriodPassed(Calibration.HATCH_RELEASE_TIME) ) {
                            hookToggle.update(!hookToggle.isInOffState());
                            hatchTime.stopAndReset();
                            hatchTime.startIfNotRunning();
                            intakeState++;
                        }
                        break;

                    case( 3 ):
                        if( hatchTime.hasPeriodPassed(Calibration.HATCH_PULL_TIME) ) {
                            sliderForward.update(sliderForward.isInOnState());
                            hatchTime.stopAndReset();
                            intakeState++;
                        }
                        break;

                    case ( 4 ):
                        intakeState = 0;
                        break;
                }
            }

        }
    }

    private class AutoCenterManager {
        private NewExtendablePIDController anglePID;
        private NewExtendablePIDController distPID;
    
        public AutoCenterManager() {
            anglePID = new NewExtendablePIDController(-0.07, 0, -0.3, new Limelight.HorizontalError(robot.limelight, 0)::pidGet,            driveManager.arcade.rotatePower::set, 0.005);
        
            anglePID.setTolerance(Calibration.LIMELIGHT_ANGLE_TOLERANCE);
            anglePID.setOutputRange(-0.7, 0.7);

            distPID = new NewExtendablePIDController(0.5, 0, 0, new Limelight.DistanceError(robot.limelight, 18)::pidGet, driveManager.arcade.rotatePower::set);
            distPID.setTolerance(Calibration.LIMELIGHT_DIST_TOLERANCE);
        }

        public void run() {
            robot.limelight.scan.activate();
            if( robot.limelight.limelightHasTargets.get() ) {
                anglePID.setEnabled(true);
                System.out.println(anglePID.get());
            } else {
                this.end();
            }
        }

        public void end() {
            anglePID.setEnabled(false);
            anglePID.reset();
            distPID.setEnabled(false);
        }
    }

    private enum CurrentDrive {
        IDLE, ARCADE, TANK, MANUAL
    }
}

