package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robot2019.modules.Dashboard;
import com._604robotics.robotnik.*;
import com._604robotics.robotnik.prefabs.devices.wrappers.RampMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

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
                backLeftTurn, backLeftDrive, backRightTurn, backRightDrive, gyroscope);

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
        public final Input<Double> spinPower;
        public final Input<Double> driveX;
        public final Input<Double> driveY;

        public RobotRelative() {
            super(SwerveDifferentialDrive.this, RobotRelative.class);

            spinPower = addInput("spinPower", 0.0, true);
            driveX = addInput("driveX", 0.0, true);
            driveY = addInput("driveY", 0.0, true);
        }

        @Override
        public void run() {
            driveBase.robotRelativeDrive(new Vector2d(driveX.get(), driveY.get()), spinPower.get());
        }

    }

    public RobotRelative robotRelative = new RobotRelative();

    public class FieldRelative extends Action {
        public final Input<Double> angle;
        public final Input<Double> driveX;
        public final Input<Double> driveY;

        public FieldRelative() {
            super(SwerveDifferentialDrive.this, FieldRelative.class);

            angle = addInput("angle", 0.0, true);
            driveX = addInput("driveX", 0.0, true);
            driveY = addInput("driveY", 0.0, true);
        }

        @Override
        public void run() {
            // NOTE: Move start-relative logic here?
            driveBase.startRelativeDrive(new Vector2d(driveX.get(), driveY.get()), angle.get());
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

    /**
     * Represents a swerve drive base that uses differential motor gearing. When the motors are run against each other,
     * the module will spin. When run in the same direction, the wheel spins. Thank mechanical for awesome gearing to
     * make this happen.
     *
     * Some basic vector math and power scaling allows for turning while driving.
     *
     * The advantage of this is the additional power that can be utilized when driving and *not* spinning in circles.
     * Imagine: a swerve drive beating a tank drive because of having two motors per wheel. Of course, this is not close
     * to being garunteed, due to having only four wheels total, but it is nice to think about.
     */
    private class SwerveDifferentialDriveBase extends RobotDriveBase {

        private final SpeedController frontLeftTurn;
        private final SpeedController frontLeftDrive;
        private final SpeedController frontRightTurn;
        private final SpeedController frontRightDrive;
        private final SpeedController backLeftTurn;
        private final SpeedController backLeftDrive;
        private final SpeedController backRightTurn;
        private final SpeedController backRightDrive;

        private final Gyro gyro;

        public SwerveDifferentialDriveBase(SpeedController frontLeftTurn, SpeedController frontLeftDrive,
                                           SpeedController frontRightTurn, SpeedController frontRightDrive,
                                           SpeedController backLeftTurn, SpeedController backLeftDrive,
                                           SpeedController backRightTurn, SpeedController backRightDrive,
                                           Gyro gyro) {

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

            this.gyro = gyro;

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

        /**
         * <p>Robot relative method of controlling a swerve drive platform.</p>
         *
         * <p>This allows for more complex maneuvers around the field, but also requires more thought by the controllers to
         * constantly remember the orientation of the robot.</p>
         *
         * <p>For example, if the robot is given a drive power of 1 and a spin of 0, the robot will drive straight relative
         * to the front of the robot. Then, if a spin is given, the robot will continue to spin until instructed to stop,
         * as there is no "end goal" of the spin.</p>
         *
         * @param vector horizontal magnitude and direction of the robot not including angle/spin <br>
         * @param spin magnitude of spin the robot should have, from -1.0 to 1.0
         */
        public void robotRelativeDrive(Vector2d vector, double spin) {
            if( spin > 1.0 ) {
                spin = 1.0;
            } else if( spin < -1.0 ) {
                spin = -1.0;
            }

            feed();
        }

        /**
         * <p>Start relative method of controlling a swerve drive platform.</p>
         *
         * <p>Each input is calculated relative to the zero orientation of the robot. This means that the inputs are
         * interpreted to align starting angle with "forwards", and allow for absolute turning angles.</p>
         *
         * <p>This is most useful to reduce effort by the driver to maintain awareness of the robot's orentation at
         * any given time. This allows for faster reactions, as the driver only has to provide the "goal" position of
         * the robot, instead of thinking of how to achieve it. However, this mode makes actions such as drifting more
         * difficult.</p>
         *
         * <p>For example, if the robot is facing 45 degrees from its starting position, but the given vector is "straight"
         * (no turning/spinning/angles away from 0), then the robot will drive in a single axis away from the start.
         * Using a field as an example, if the robot starts in the center pushed against the alliance wall, then
         * independent of the robot's orientation it will drive towards the opposite wall given a vector with angle 0.</p>
         *
         * @param vector horizontal magnitude and direction of the robot not including angle/spin <br>
         * @param angle desired angle of the robot relative to starting angle in radians
         */
        public void startRelativeDrive(Vector2d vector, double angle) {
            if( gyro == null ) {
                throw new RuntimeException("Gyro needed for relative swerve drive");
            }

            // Corrects to counterclockwise positive, keeps in 1 circle, and converts to radians
            double currentAngle = Math.toRadians((gyro.getAngle() - 180) % 360);
            angle = angle % (Math.PI * 2); // Keeps angles within one circle

            // Calculates current difference in angle

            feed();
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
