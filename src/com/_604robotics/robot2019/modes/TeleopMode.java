package com._604robotics.robot2019.modes;

import com._604robotics.marionette.InputPlayer;
import com._604robotics.marionette.InputRecorder;
import com._604robotics.marionette.InputRecording;
import com._604robotics.marionette.MarionetteJoystick;
import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.modules.Arm;
import com._604robotics.robot2019.modules.Drive;
import com._604robotics.robot2019.modules.Intake;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.controller.ExtendablePIDController;
import com._604robotics.robotnik.prefabs.flow.Toggle;
import com._604robotics.robotnik.prefabs.inputcontroller.xbox.XboxController;
import com._604robotics.robotnik.prefabs.modules.Limelight;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;

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

            if (driverLeftJoystickButton) {
                leftY *= 0.8;
                rightY *= 0.8;
                rightX *= 0.8;
            }

            inverted.update(driverLeftBumper);
            robot.dashboard.XboxFlipped.set(inverted.isInOnState());
            if (inverted.isInOnState()) {            // Flip values if xbox inverted
                leftY*=-1;
                rightY*=-1;
            }

            if( driverStart ) {
                robot.powermonitor.updateCompressor(false);
                if ( robot.slider.isForward.get() ) {
                    hatchManager.sliderForward.update(true);
                }
                armManager.disableArm = true;
                robot.arm.setpoint.setpoint.set(Calibration.Arm.LOW_SETPOINT);
                robot.arm.setpoint.activate();
                if ( driverDPad ) {
                    robot.tilter.tilt.activate();
                } else if ( driverBack ) {
                    robot.tilter.retract.activate();
                } else {
                    robot.tilter.stow.activate();
                }   
            } else {
                armManager.disableArm = false;
                robot.powermonitor.updateCompressor(true);
                robot.tilter.stow.activate();
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

            if( driverRightBumper ){
                robot.limelight.scan.activate();
                arcade.activate();

                if( robot.limelight.limelightHasTargets.get() ){
                    arcade.movePower.set(leftY);
                    autoCenterManager.run();
                } else {
                    robot.limelight.scan.activate();
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
                    }
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
                }
            }
        }
    }


    private class IntakeManager {
        private final Intake.Idle idle;
        private final Intake.Speed speed;

        public IntakeManager() {
            idle = robot.intake.new Idle();
            speed = robot.intake.new Speed();
        }

        public void run() {
            if( driverRightTrigger != 0.0 /*&& !speed.getState()*/) {
                speed.set(-driverRightTrigger); // Intake
                //Negative is Intake
            } else if( driverLeftTrigger != 0.0 ){
                speed.set(driverLeftTrigger); // Outtake
            } else if( manipLeftTrigger != 0.0 /*&& !speed.getState()*/) {
                speed.set( -manipLeftTrigger);
            } else if( manipRightTrigger != 0.0 ) {
                speed.set( manipRightTrigger);
            } else if( manipDPad ) {
                speed.set(1); //Force spit
            } else {
                idle.activate();
            }

        }
    }

    private class ArmManager {
        private Arm arm;
        protected boolean disableArm;

        public ArmManager() {
            arm = robot.arm;
            disableArm = false;
        }

        public void run() {
            if ( manipBack ) {
                arm.resetEncoder();
            }

            if ( !disableArm ) {
                if ( !robot.slider.isForward.get() ) {
                    // Check setpoints
                    if( manipA ) {
                        // Low position
                        arm.setpoint.setpoint.set(Calibration.Arm.LOW_SETPOINT);
                        arm.setpoint.activate();
                    } else if( manipB ) {
                        // Ball place position
                        arm.setpoint.setpoint.set(Calibration.Arm.OUTPUT_SETPOINT);
                        arm.setpoint.activate();
                    } else if( manipY ) {
                        // Vertical position
                        arm.setpoint.setpoint.set(Calibration.Arm.VERTICAL_POSITION);
                        arm.setpoint.activate();
                    } else if( manipX ) {
                        // Vertical position
                        arm.setpoint.setpoint.set(Calibration.Arm.ROCKET_SETPOINT);
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

                        if ( manipBack ){
                            arm.hold.activate();
                        }
                    }
                } else {
                    arm.hold.activate();
                }
            }

        }

    }

    private class HatchManager {
        private Toggle useAuto;
        private Toggle hookToggle;
        protected Toggle sliderForward;
        private SmartTimer hatchTime;
        private int autoState = 0;
        private int intakeState = 0;

		private double start;

        public HatchManager() {
            useAuto = new Toggle(false);
            hookToggle = new Toggle(false); // Assuming the piston is in the held state to start
            sliderForward = new Toggle(false); // Not extended initially
            hatchTime = new SmartTimer();

			start = System.currentTimeMillis();
        }

        public void run() {

            if( driverA ) {
                hookToggle.update(driverA);
            } else if ( manipLeftBumper ){
                hookToggle.update( manipLeftBumper );
            } else {
                hookToggle.update(false);
            }

            if ( hookToggle.isInOffState() ) {
                robot.hook.hold.activate();

            } else if ( hookToggle.isInOnState() ) {
                robot.hook.release.activate();

            }


            if(robot.arm.leftEncoderClicks.get() <= 1560) {
                if( driverY ) {
                    sliderForward.update( driverY );
                } else if( manipRightBumper ) {
                    sliderForward.update( manipRightBumper);
                } else {
                    sliderForward.update(false);
                }

                if( sliderForward.isInOnState()) {
                        robot.slider.front.activate();
                        robot.limelight.limelightLED.set(1);
                } else if( sliderForward.isInOffState() ) {
                    robot.slider.back.activate();
                    robot.limelight.limelightLED.set(robot.dashboard.limelightLEDState.get().ordinal());

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
                    if( hatchTime.hasPeriodPassed(Calibration.PUSH_TIME) ) {
                        hookToggle.update(!hookToggle.isInOnState());
                        hatchTime.stopAndReset();
                        hatchTime.startIfNotRunning();
                        autoState++;
                    }
                    break;

                case( 3 ):
                    if( hatchTime.hasPeriodPassed(Calibration.BACK_TIME) ) {
                        sliderForward.update(sliderForward.isInOnState());
                        hatchTime.stopAndReset();
                        autoState++;

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
                        if( hatchTime.hasPeriodPassed(Calibration.RELEASE_TIME) ) {
                            hookToggle.update(!hookToggle.isInOffState());
                            hatchTime.stopAndReset();
                            hatchTime.startIfNotRunning();
                            intakeState++;
                        }
                        break;

                    case( 3 ):
                        if( hatchTime.hasPeriodPassed(Calibration.PULL_TIME) ) {
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
        private ExtendablePIDController anglePID;
        private ExtendablePIDController distPID;
        private PIDOutput rotation;
        private PIDOutput drive;

        public AutoCenterManager() {
            rotation = new PIDOutput() {
                @Override
                public synchronized void pidWrite(double output) {
                    if( output >= 0.535 ) {
                        output = 0.535;
                    } else if( output <= -0.535 ) {
                        output = -0.535;
                    }
                    driveManager.arcade.rotatePower.set(output);
                }
            };
            drive = new PIDOutput() {
                @Override
                public synchronized void pidWrite(double output) {
                    driveManager.arcade.movePower.set(output);
                }
            };
            anglePID = new ExtendablePIDController(-0.05, 0, -0.3, new Limelight.HorizontalError(robot.limelight,0), rotation, 0.025);
            anglePID.setAbsoluteTolerance(Calibration.LIMELIGHT_ANGLE_TOLERANCE);
            distPID = new ExtendablePIDController(0.5, 0, 0, new Limelight.DistanceError(robot.limelight, 18), drive);
            distPID.setAbsoluteTolerance(Calibration.LIMELIGHT_DIST_TOLERANCE);
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
        IDLE, ARCADE, TANK
    }
}

