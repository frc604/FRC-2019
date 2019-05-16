package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.prefabs.devices.SwerveUnit;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class SwerveDrive extends Module {
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
    private final SwerveUnit frontLeft;
    private final SwerveUnit frontRight;
    private final SwerveUnit backLeft;
    private final SwerveUnit backRight;
    //</editor-fold>

    final SwerveDriveBase driveBase;

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
    public final Output<Double> frontLeftTurnClicks;
    public final Output<Double> frontLeftDriveClicks;
    public final Output<Double> frontRightTurnClicks;
    public final Output<Double> frontRightDriveClicks;
    public final Output<Double> backLeftTurnClicks;
    public final Output<Double> backLeftDriveClicks;
    public final Output<Double> backRightTurnClicks;
    public final Output<Double> backRightDriveClicks;
    //</editor-fold>

    public SwerveDrive(SwerveUnit frontLeft, SwerveUnit frontRight, SwerveUnit backLeft, SwerveUnit backRight,
                       Gyro gyroscope, Accelerometer accelerometer) {
        super(SwerveDrive.class);

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.driveBase = new SwerveDriveBase(frontLeft, frontRight, backLeft, backRight, gyroscope);

        this.gyroscope = gyroscope;
        this.accelerometer = accelerometer;

        this.xAccel = addOutput("X Accelleration", accelerometer::getX);
        this.yAccel = addOutput("Y Accelleration", accelerometer::getY);
        this.zAccel = addOutput("Z Accelleration", accelerometer::getZ);
        this.horizGyro = addOutput("Gyro", gyroscope::getAngle);

        this.frontLeftTurnClicks = addOutput("Front Left Turn Clicks", frontLeft::getTurnClicks);
        this.frontLeftDriveClicks = addOutput("Front Left Drive Clicks", frontLeft::getDriveClicks);
        this.frontRightTurnClicks = addOutput("Front Left Right Clicks", frontRight::getTurnClicks);
        this.frontRightDriveClicks = addOutput("Front Right Drive Clicks", frontRight::getDriveClicks);
        this.backLeftTurnClicks = addOutput("Back Left Turn Clicks", backLeft::getTurnClicks);
        this.backLeftDriveClicks = addOutput("Back Left Drive Clicks", backLeft::getDriveClicks);
        this.backRightTurnClicks = addOutput("Back Left Right Clicks", backRight::getTurnClicks);
        this.backRightDriveClicks = addOutput("Back Right Drive Clicks", backRight::getDriveClicks);

        this.driveBase.setDeadband(0.04);

        setDefaultAction(idle);
    }

    public SwerveDrive(SwerveUnit frontLeft, SwerveUnit frontRight, SwerveUnit backLeft, SwerveUnit backRight,
                       Gyro gyroscope) {
        this(frontLeft, frontRight, backLeft, backRight, gyroscope, new BuiltInAccelerometer());
    }

    public class Idle extends Action {
        public Idle() {
            super(SwerveDrive.this, Idle.class);
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
            super(SwerveDrive.this, RobotRelative.class);

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
            super(SwerveDrive.this, FieldRelative.class);

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
        frontLeft.resetDriveEncoder();
        frontLeft.resetTurnEncoder();
        frontRight.resetDriveEncoder();
        frontRight.resetTurnEncoder();
        backLeft.resetDriveEncoder();
        backLeft.resetTurnEncoder();
        backRight.resetDriveEncoder();
        backRight.resetTurnEncoder();

        gyroscope.reset();
    }

    /**
     * Represents a swerve drive base that uses 4 modules total.
     */
    private class SwerveDriveBase extends RobotDriveBase {

        private final SwerveUnit frontLeft;
        private final SwerveUnit frontRight;
        private final SwerveUnit backLeft;
        private final SwerveUnit backRight;

        private final Gyro gyro;

        public SwerveDriveBase(SwerveUnit frontLeft, SwerveUnit frontRight, SwerveUnit backLeft,
                               SwerveUnit backRight, Gyro gyro) {

            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;

            this.gyro = gyro;

            addChild(frontLeft);
            addChild(frontRight);
            addChild(backLeft);
            addChild(backRight);

            setName("SwerveDriveBase");
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

            // Used to determine the preference for the angle of the wheels, when trading between driving in a
            // straight line, and spinning in place
            double drivePower = Math.min(vector.magnitude(), vector.magnitude() / (vector.magnitude() + spin));
            double spinPower = Math.min(spin, spin / (vector.magnitude() + spin));

            // If vector = 0, then spin in place
            // Positive is clockwise, from top-down
            frontLeft.setAngle(Math.toRadians(45));
            frontRight.setAngle(Math.toRadians(135));
            backLeft.setAngle(Math.toRadians(225));
            backRight.setAngle(Math.toRadians(315));
            setAllDrive(spin);

            // If spin = 0, then align to vector
            /* Controller axis example:
            -+#++
            #####
            --#+- */
            // +,+ = 45
            // +,- = 135
            // -,+ = 225
            // -,- = 315
            double vectorAngle = Math.atan2(vector.y, vector.x);
            frontLeft.setAngle(vectorAngle);
            frontRight.setAngle(vectorAngle);
            backLeft.setAngle(vectorAngle);
            backRight.setAngle(vectorAngle);
            setAllDrive(vector.magnitude());

            // So, to get both, we need to use a ratio. Such as the power.

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
            if( gyro != null ) {
                robotRelativeDrive(vector, angle); // Default to robot relative if no gyro
                throw new RuntimeException("Gyro needed for relative swerve drive");
            }

            // Corrects to counterclockwise positive, keeps in 1 circle, and converts to radians
            double currentAngle = Math.toRadians((gyro.getAngle() - 180) % 360);
            angle = angle % (Math.PI * 2); // Keeps angles within one circle

            // Calculates current difference in angle

            feed();
        }

        /**
         * Useful for autonomous turns, may not be useful...
         *
         * @param center imaginary "center" of the circle to revolve around
         * @param endAngle direction the robot should face relative to the start
         * @param clicks distance to travel in encoder clicks
         */
        public void arbitraryCircleDrive(Vector2d center, double endAngle, double clicks) {

        }

        @Override
        public void stopMotor() {
            frontLeft.stopMotor();
            frontRight.stopMotor();
            backLeft.stopMotor();
            backRight.stopMotor();

            feed();
        }

        private void setAllDrive(double power) {
            frontLeft.setDrive(power);
            frontRight.setDrive(power);
            backLeft.setDrive(power);
            backRight.setAngle(power);
        }

        @Override
        public String getDescription() {
            return "SwerveDriveBase";
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("DifferentialDrive");
            builder.setSafeState(this::stopMotor);
            builder.setActuator(true);

            builder.addDoubleProperty("Front Left Turn Angle", frontLeft::getAngle, frontLeft::setAngle);
            builder.addDoubleProperty("Front Left Drive Power", frontLeft::getDrive, frontLeft::setDrive);
            builder.addDoubleProperty("Front Right Turn Angle", frontRight::getAngle, frontRight::setAngle);
            builder.addDoubleProperty("Front Right Drive Power", frontRight::getDrive, frontRight::setDrive);
            builder.addDoubleProperty("Back Left Turn Angle", backLeft::getAngle, backLeft::setAngle);
            builder.addDoubleProperty("Back Left Drive Power", backLeft::getDrive, backLeft::setDrive);
            builder.addDoubleProperty("Back Right Turn Angle", backRight::getAngle, backRight::setAngle);
            builder.addDoubleProperty("Back Right Drive Power", backRight::getDrive, backRight::setDrive);

        }
    }
}
