package com._604robotics.robotnik.utils;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import com._604robotics.robotnik.prefabs.controller.ExtendablePIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

import com._604robotics.robotnik.utils.geometry.Rotation2d;
import com._604robotics.robotnik.utils.kinematics.DifferentialDriveKinematics;
import com._604robotics.robotnik.utils.kinematics.DifferentialDriveOdometry;
import com._604robotics.robotnik.utils.kinematics.DifferentialDriveWheelSpeeds;

public class DrivetrainOdometry {

    private DrivetrainOdometry() {
    
    }

    public static class DriveOdometry {

        public ExtendablePIDController leftPIDController;
        public ExtendablePIDController rightPIDController;

        private final DifferentialDriveKinematics driveKinematics;
        private final DifferentialDriveOdometry driveOdometry;

        private double leftDrivePower;
        private double rightDrivePower;

        public final double kMaxSpeed;
        public final double kMaxAngularSpeed;
        public final double kTrackWidth; 
        public final double kWheelRadius;
        public final int kEncoderResolution;
        public final AnalogGyro gyro;
        public final Encoder rightEncoder;
        public final Encoder leftEncoder;


        public PIDSource leftEncoderPidSource = new PIDSource() {

            private PIDSourceType type;

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {type = pidSource;}

            @Override
            public PIDSourceType getPIDSourceType() {return type;}

            @Override
            public double pidGet() {
                return leftEncoder.getRate();
            }
        };

        public PIDOutput leftPidOutput = new PIDOutput() {

            @Override
            public synchronized void pidWrite(double output) {
                leftDrivePower = output;
            }
        };

        public PIDSource rightEncoderPidSource = new PIDSource() {

            private PIDSourceType type;

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {type = pidSource;}

            @Override
            public PIDSourceType getPIDSourceType() {return type;}

            @Override
            public double pidGet() {
                return leftEncoder.getRate();
            }
        };

        public PIDOutput rightPidOutput = new PIDOutput() {

            @Override
            public synchronized void pidWrite(double output) {
                rightDrivePower = output;
            }
        };        

        public DriveOdometry(double kMaxSpeed, double kMaxAngularSpeed, 
            double kTrackWidth, double kWheelRadius, int kEncoderResolution, 
            AnalogGyro gyro, Encoder rightEncoder, Encoder leftEncoder,
            double Kpr, double Kir, double Kdr, double Kpl, double Kil, double Kdl) {
            this.kMaxSpeed = kMaxSpeed;
            this.kMaxAngularSpeed = kMaxAngularSpeed;
            this.kTrackWidth = kTrackWidth;
            this.kWheelRadius = kWheelRadius;
            this.kEncoderResolution = kEncoderResolution;
            this.gyro = gyro;
            this.rightEncoder = rightEncoder;
            this.leftEncoder = leftEncoder;


            leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
            rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        
            leftPIDController = new ExtendablePIDController(Kpl, Kil, Kdl, leftEncoderPidSource, leftPidOutput);
            rightPIDController = new ExtendablePIDController(Kpr, Kir, Kdr, rightEncoderPidSource, rightPidOutput);
            
            driveKinematics = new DifferentialDriveKinematics(kTrackWidth);
            driveOdometry = new DifferentialDriveOdometry(driveKinematics);

        }
            
        public Rotation2d getAngle() {
            return Rotation2d.fromDegrees(-gyro.getAngle());

        }

        public DifferentialDriveWheelSpeeds getcurrentDriveWheelSpeeds() {
            return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());

        }

        public double leftDriveWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
            leftPIDController.setSetpoint(speeds.leftMetersPerSecond);
            leftPIDController.setEnabled(true);
            return leftDrivePower;
            
        }

        public double rightDriveWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
            rightPIDController.setSetpoint(speeds.rightMetersPerSecond);
            rightPIDController.setEnabled(true);
            return rightDrivePower;
        }

        public void updateOdometry() {
            driveOdometry.update(getAngle(), getcurrentDriveWheelSpeeds());
          }
    }

}