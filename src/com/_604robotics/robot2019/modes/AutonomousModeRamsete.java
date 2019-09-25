package com._604robotics.robot2019.modes;

import com._604robotics.marionette.InputRecording;
import com._604robotics.robot2019.Robot2019;
import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.modules.Drive;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.controller.RamseteController;
import com._604robotics.robotnik.prefabs.controller.SingleThreadPIDController;
import com._604robotics.robotnik.utils.geometry.Pose2d;
import com._604robotics.robotnik.utils.geometry.Rotation2d;
import com._604robotics.robotnik.utils.kinematics.ChassisSpeeds;
import com._604robotics.robotnik.utils.kinematics.DifferentialDriveKinematics;
import com._604robotics.robotnik.utils.kinematics.DifferentialDriveOdometry;
import com._604robotics.robotnik.utils.kinematics.DifferentialDriveWheelSpeeds;
import com._604robotics.robotnik.utils.trajectory.Trajectory;
import com._604robotics.robotnik.utils.trajectory.Trajectory.State;
import com._604robotics.robotnik.utils.trajectory.TrajectoryGenerator;
import com._604robotics.robotnik.utils.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.Timer;

import java.io.IOException;
import java.util.List;

public class AutonomousModeRamsete extends Coordinator {
    private static final Logger logger = new Logger(AutonomousModeRamsete.class);

    private final com._604robotics.robot2019.Robot2019 robot;
    public String primaryFileName;
    public String secondaryFileName;
    private Coordinator selectedModeMacro;

    public AutonomousModeRamsete(Robot2019 robot) {
        this.robot = robot;
    }

    protected final class TankOdometryHandler {
        private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Calibration.DRIVE_PROPERTIES.trackWidth);
        private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(kinematics);
        private Drive.TankDrive tankDrive = robot.drive.new TankDrive(false);
        private SingleThreadPIDController leftController;
        private SingleThreadPIDController rightController;


        public TankOdometryHandler() {
            tankDrive.activate();

            leftController = new SingleThreadPIDController(1, 0, 0);
            rightController = new SingleThreadPIDController(1, 0, 0);
        }

        public double getLeftError() {
            return leftController.getPositionError();
        }

        public double getRightError() {
            return rightController.getPositionError();
        }

        public Rotation2d getAngle() {
            return Rotation2d.fromDegrees(-robot.drive.gyroAngle.get());
        }

        public DifferentialDriveWheelSpeeds getCurrentSpeeds() {
            return new DifferentialDriveWheelSpeeds(robot.drive.leftClickRate.get(), robot.drive.rightClickRate.get());

        }

        public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
            double leftOutput = leftController.calculate(robot.drive.leftClickRate.get(),
                    speeds.leftMetersPerSecond);
            double rightOutput = rightController.calculate(robot.drive.rightClickRate.get(),
                    speeds.rightMetersPerSecond);

            tankDrive.leftPower.set(leftOutput);
            tankDrive.rightPower.set(rightOutput);

        }

        public void drive(double speed, double rot) {
            DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0, rot));
            setSpeeds(wheelSpeeds);
        }

        public Pose2d updateOdometry(double time) {
            return odometry.updateWithTime(time, getAngle(), getCurrentSpeeds());
        }

        public void reset(Pose2d pose) {
            odometry.resetPosition(pose);
        }

    }

    private class PathFollower extends Coordinator {
        private Trajectory path;
        private Drive.TankDrive tankDrive;
        private RamseteController controller;
        private PathFollowTask followTask;


        public PathFollower(List<Pose2d> waypoints, List<TrajectoryConstraint> constraints,
            double startVelocity, double endVelocity, double maxVelocity, double maxAccel) {
            this(TrajectoryGenerator.generateTrajectory(
                    waypoints,
                    constraints,
                    startVelocity,
                    endVelocity,
                    maxVelocity,
                    maxAccel,
                    true
            ));

        }

        public PathFollower(Trajectory path) {
            this.path = path;
            tankDrive = robot.drive.new TankDrive(false);

        }

        private class PathFollowTask {
			private Pose2d robotPose;
        	private State desiredState;
            private ChassisSpeeds output;
            private TankOdometryHandler tank;
            private double dt;
            private double initialTime;


            PathFollowTask(double b, double zeta, double dt) {
                this.dt = dt;
                controller = new RamseteController(b, zeta);
                tank = new TankOdometryHandler();
				tank.reset(path.sample(0).poseMeters);//Getting initial robot pose
                initialTime = Timer.getFPGATimestamp();

            }

            public void run() {
                tankDrive.activate();
                robotPose = tank.updateOdometry(Timer.getFPGATimestamp() - initialTime);

                desiredState = path.sample((Timer.getFPGATimestamp() - initialTime) + dt);

                output = controller.calculate(robotPose, desiredState);

                tank.drive(output.vxMetersPerSecond, output.omegaRadiansPerSecond);

            }

            public double getTimePassed() {
                return (Timer.getFPGATimestamp() - initialTime);
            }

            public void resetTime() {
                initialTime = Timer.getFPGATimestamp();
            }

        }

        @Override
        //begin() is run when Coordinator is activated
        public void begin() {
            robot.drive.resetSensors();
            followTask = new PathFollowTask(Calibration.RamsetePathFollower.b, 
                Calibration.RamsetePathFollower.zeta, Calibration.RamsetePathFollower.dt);

        }

        //Following begin run() is run
        public boolean run() {
            // getTimePassed() Returns time passed from when PathFollowTask object created 
            if (followTask.getTimePassed() >= path.getTotalTimeSeconds()) {
                return true; //Interrupts Coordinator run() loop if true; goes to end()

            } else {
                followTask.run();
                return false;

            }
        }

        @Override
		public void end() {
            System.out.println("ERROR: end");
            followTask.resetTime();

		}

    }
}
