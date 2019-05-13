package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.*;
import com._604robotics.robotnik.prefabs.devices.wrappers.RampMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import java.util.StringJoiner;

public class SwerveDifferentialDrive extends Module {
    /* To copy paste:
        frontLeftTurn
        frontLeftDrive
        frontRightTurn
        frontRightDrive
        backLeftTurn
        backLeftDrive
        backRightTurn
        backRightDrive
     */

    //<editor-fold desc="Motors"
    private final RampMotor frontLeftTurn;
    private final RampMotor frontLeftDrive;
    private final RampMotor frontRightTurn;
    private final RampMotor frontRightDrive;
    private final RampMotor backLeftTurn;
    private final RampMotor backLeftDrive;
    private final RampMotor backRightTurn;
    private final RampMotor backRightDrive;
    //</editor-fold>

    final SwerveDifferentialDriveBase driveBase;

    //<editor-fold desc="Encoders"
    private final Encoder frontLeftTurnEncoder;
    private final Encoder frontLeftDriveEncoder;
    private final Encoder frontRightTurnEncoder;
    private final Encoder frontRightDriveEncoder;
    private final Encoder backLeftTurnEncoder;
    private final Encoder backLeftDriveEncoder;
    private final Encoder backRightTurnEncoder;
    private final Encoder backRightDriveEncoder;
    //</editor-fold>

    //<editor-fold desc="Sensors"
    private final Gyro gyroscope;
    private final Accelerometer accelerometer;
    //</editor-fold>

    // Other values
    private double deadband = 0.04;

    //<editor-fold desc="Sensor Outputs"
    public final Output<Double> xAccel;
    public final Output<Double> yAccel;
    public final Output<Double> zAccel;
    public final Output<Double> horizGyro;
    //</editor-fold>

    //<editor-fold desc="Drive Encoder Outputs"
    public final Output<Integer> frontLeftTurnClicks;
    public final Output<Integer> frontLeftDriveClicks;
    public final Output<Integer> frontRightTurnClicks;
    public final Output<Integer> frontRightDriveClicks;
    public final Output<Integer> backLeftTurnClicks;
    public final Output<Integer> backLeftDriveClicks;
    public final Output<Integer> backRightTurnClicks;
    public final Output<Integer> backRightDriveClicks;
    //</editor-fold>

    public SwerveDifferentialDrive( RampMotor frontLeftTurn, RampMotor frontLeftDrive,
                                    RampMotor frontRightTurn, RampMotor frontRightDrive,
                                    RampMotor backLeftTurn, RampMotor backLeftDrive,
                                    RampMotor backRightTurn, RampMotor backRightDrive,
                                    Encoder frontLeftTurnEncoder, Encoder frontLeftDriveEncoder,
                                    Encoder frontRightTurnEncoder, Encoder frontRightDriveEncoder,
                                    Encoder backLeftTurnEncoder, Encoder backLeftDriveEncoder,
                                    Encoder backRightTurnEncoder, Encoder backRightDriveEncoder,
                                    Gyro gyroscope, Accelerometer accelerometer ) {
        super(SwerveDifferentialDrive.class);

        this.frontLeftTurn = frontLeftTurn;
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightTurn = frontRightTurn;
        this.frontRightDrive = frontRightDrive;
        this.backLeftTurn = backLeftTurn;
        this.backLeftDrive = backLeftDrive;
        this.backRightTurn = backRightTurn;
        this.backRightDrive = backRightDrive;

        this.driveBase = new SwerveDifferentialDriveBase(frontLeftTurn, frontLeftDrive, frontRightTurn, frontRightDrive,
                backLeftTurn, backLeftDrive, backRightTurn, backRightDrive);

        this.frontLeftTurnEncoder = frontLeftTurnEncoder;
        this.frontLeftDriveEncoder = frontLeftDriveEncoder;
        this.frontRightTurnEncoder = frontRightTurnEncoder;
        this.frontRightDriveEncoder = frontRightDriveEncoder;
        this.backLeftTurnEncoder = backLeftTurnEncoder;
        this.backLeftDriveEncoder = backLeftDriveEncoder;
        this.backRightTurnEncoder = backRightTurnEncoder;
        this.backRightDriveEncoder = backRightDriveEncoder;

        this.gyroscope = gyroscope;
        this.accelerometer = accelerometer;

        this.xAccel = addOutput("X Accelleration", accelerometer::getX);
        this.yAccel = addOutput("Y Accelleration", accelerometer::getY);
        this.zAccel = addOutput("Z Accelleration", accelerometer::getZ);
        this.horizGyro = addOutput("Gyro", gyroscope::getAngle);

        this.frontLeftTurnClicks = addOutput("Front Left Turn Clicks", frontLeftTurnEncoder::get);
        this.frontLeftDriveClicks = addOutput("Front Left Drive Clicks", frontLeftDriveEncoder::get);
        this.frontRightTurnClicks = addOutput("Front Left Right Clicks", frontRightTurnEncoder::get);
        this.frontRightDriveClicks = addOutput("Front Right Drive Clicks", frontRightDriveEncoder::get);
        this.backLeftTurnClicks = addOutput("Back Left Turn Clicks", backLeftTurnEncoder::get);
        this.backLeftDriveClicks = addOutput("Back Left Drive Clicks", backLeftDriveEncoder::get);
        this.backRightTurnClicks = addOutput("Back Left Right Clicks", backRightTurnEncoder::get);
        this.backRightDriveClicks = addOutput("Back Right Drive Clicks", backRightDriveEncoder::get);

        this.driveBase.setDeadband(0.04);

        setDefaultAction(idle);
    }

    public SwerveDifferentialDrive( SpeedController frontLeftTurn, SpeedController frontLeftDrive,
                                    SpeedController frontRightTurn, SpeedController frontRightDrive,
                                    SpeedController backLeftTurn, SpeedController backLeftDrive,
                                    SpeedController backRightTurn, SpeedController backRightDrive,
                                    Encoder frontLeftTurnEncoder, Encoder frontLeftDriveEncoder,
                                    Encoder frontRightTurnEncoder, Encoder frontRightDriveEncoder,
                                    Encoder backLeftTurnEncoder, Encoder backLeftDriveEncoder,
                                    Encoder backRightTurnEncoder, Encoder backRightDriveEncoder,
                                    Gyro gyroscope, Accelerometer accelerometer ) {
        this(new RampMotor(frontLeftTurn), new RampMotor(frontLeftDrive),
                new RampMotor(frontRightTurn), new RampMotor(frontRightDrive),
                new RampMotor(backLeftTurn), new RampMotor(backLeftDrive),
                new RampMotor(backRightTurn), new RampMotor(backRightDrive),
                frontLeftTurnEncoder, frontLeftDriveEncoder, frontRightTurnEncoder, frontRightDriveEncoder,
                backLeftTurnEncoder, backLeftDriveEncoder, backRightTurnEncoder, backRightDriveEncoder,
                gyroscope, accelerometer);
    }


    public class Idle extends Action {
        public Idle() {
            super(SwerveDifferentialDrive.this, Idle.class);
        }

        @Override
        public void run() {
            driveBase.stopMotor();
        }
    }

    public Idle idle = new Idle();

    public class RobotRelative extends Action {
        public final Input<Double> angleDirection;
        public final Input<Double> anglePower;
        public final Input<Double> driveDirection;
        public final Input<Double> drivePower;

        public RobotRelative() {
            super(SwerveDifferentialDrive.this, RobotRelative.class);

            angleDirection = addInput("angleDirection", 0.0, true);
            anglePower = addInput("anglePower", 0.0, true);
            driveDirection = addInput("driveDirection", 0.0, true);
            drivePower = addInput("drivePower", 0.0, true);
        }

    }

    public RobotRelative robotRelative = new RobotRelative();

    public class FieldRelative extends Action {
        public FieldRelative() {
            super(SwerveDifferentialDrive.this, FieldRelative.class);
        }

    }

    public FieldRelative fieldRelative = new FieldRelative();

    public synchronized void resetSensors() {
        frontLeftTurnEncoder.reset();
        frontLeftDriveEncoder.reset();
        frontRightTurnEncoder.reset();
        frontRightDriveEncoder.reset();
        backLeftTurnEncoder.reset();
        backLeftDriveEncoder.reset();
        backRightTurnEncoder.reset();
        backRightDriveEncoder.reset();

        gyroscope.reset();
    }

    private class SwerveDifferentialDriveBase extends RobotDriveBase {

        private final SpeedController frontLeftTurn;
        private final SpeedController frontLeftDrive;
        private final SpeedController frontRightTurn;
        private final SpeedController frontRightDrive;
        private final SpeedController backLeftTurn;
        private final SpeedController backLeftDrive;
        private final SpeedController backRightTurn;
        private final SpeedController backRightDrive;

        public SwerveDifferentialDriveBase(SpeedController frontLeftTurn, SpeedController frontLeftDrive,
                                           SpeedController frontRightTurn, SpeedController frontRightDrive,
                                           SpeedController backLeftTurn, SpeedController backLeftDrive,
                                           SpeedController backRightTurn, SpeedController backRightDrive) {
            verifyMotors(frontLeftTurn, frontLeftDrive, frontRightTurn, frontRightDrive, backLeftTurn, backLeftDrive,
                    backRightTurn, backRightDrive);

            this.frontLeftTurn = frontLeftTurn;
            this.frontLeftDrive = frontLeftDrive;
            this.frontRightTurn = frontRightTurn;
            this.frontRightDrive = frontRightDrive;
            this.backLeftTurn = backLeftTurn;
            this.backLeftDrive = backLeftTurn;
            this.backRightTurn = backLeftTurn;
            this.backRightDrive = backLeftTurn;

            addChild(frontLeftTurn);
            addChild(frontLeftDrive);
            addChild(frontRightTurn);
            addChild(frontRightDrive);
            addChild(backLeftTurn);
            addChild(backLeftDrive);
            addChild(backRightTurn);
            addChild(backRightDrive);

            setName("SwerveDifferentialDrive");
        }

        @Override
        public void stopMotor() {
            frontLeftTurn.stopMotor();
            frontLeftDrive.stopMotor();
            frontRightTurn.stopMotor();
            frontRightDrive.stopMotor();
            backLeftTurn.stopMotor();
            backLeftDrive.stopMotor();
            backRightTurn.stopMotor();
            backRightDrive.stopMotor();

            feed();
        }

        private void verifyMotors(SpeedController... motors) {
            int numNull = 0;

            for( SpeedController s : motors ) {
                if( s == null ) {
                    numNull++;
                }
            }

            if( numNull > 0 ) {
                throw new NullPointerException(numNull + " swerve motors");
            }
        }

        @Override
        public String getDescription() {
            return "SwerveDifferentialDrive";
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("DifferentialDrive");
            builder.setSafeState(this::stopMotor);
            builder.setActuator(true);

            builder.addDoubleProperty("Front Left Turn Speed", frontLeftTurn::get, frontLeftTurn::set);
            builder.addDoubleProperty("Front Left Drive Speed", frontLeftDrive::get, frontLeftDrive::set);
            builder.addDoubleProperty("Front Right Turn Speed", frontRightTurn::get, frontRightTurn::set);
            builder.addDoubleProperty("Front Right Drive Speed", frontRightDrive::get, frontRightDrive::set);
            builder.addDoubleProperty("Back Left Turn Speed", backLeftTurn::get, backLeftTurn::set);
            builder.addDoubleProperty("Back Left Drive Speed", backLeftDrive::get, backLeftDrive::set);
            builder.addDoubleProperty("Back Right Turn Speed", backRightTurn::get, backRightTurn::set);
            builder.addDoubleProperty("Back Right Drive Speed", backRightDrive::get, backRightDrive::set);

        }
    }
}
