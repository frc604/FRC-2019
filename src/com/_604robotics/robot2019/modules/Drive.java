package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.devices.wrappers.RampMotor;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class Drive extends Module {
  private final RampMotor m_frontLeft =
      new RampMotor(new WPI_VictorSPX(Ports.DRIVE_FRONT_LEFT_MOTOR), Calibration.DRIVE_MOTOR_RAMP);
  private final RampMotor m_middleLeft =
      new RampMotor(new WPI_VictorSPX(Ports.DRIVE_MIDDLE_LEFT_MOTOR), Calibration.DRIVE_MOTOR_RAMP);
  private final RampMotor m_rearLeft =
      new RampMotor(new WPI_VictorSPX(Ports.DRIVE_REAR_LEFT_MOTOR), Calibration.DRIVE_MOTOR_RAMP);
  private final SpeedControllerGroup m_left =
      new SpeedControllerGroup(m_frontLeft, m_middleLeft, m_rearLeft);

  private final RampMotor m_frontRight =
      new RampMotor(new WPI_VictorSPX(Ports.DRIVE_FRONT_RIGHT_MOTOR), Calibration.DRIVE_MOTOR_RAMP);
  private final RampMotor m_middleRight =
      new RampMotor(
          new WPI_VictorSPX(Ports.DRIVE_MIDDLE_RIGHT_MOTOR), Calibration.DRIVE_MOTOR_RAMP);
  private final RampMotor m_rearRight =
      new RampMotor(new WPI_VictorSPX(Ports.DRIVE_REAR_RIGHT_MOTOR), Calibration.DRIVE_MOTOR_RAMP);
  private final SpeedControllerGroup m_right =
      new SpeedControllerGroup(m_frontRight, m_middleRight, m_rearRight);

  DifferentialDrive robotDrive = new DifferentialDrive(m_left, m_right);

  // Reversed from previously due to new mountings
  private final Encoder encoderLeft = new Encoder(Ports.ENCODER_LEFT_A, Ports.ENCODER_LEFT_B, true);
  private final Encoder encoderRight =
      new Encoder(Ports.ENCODER_RIGHT_A, Ports.ENCODER_RIGHT_B, false);

  private final AnalogGyro horizGyro = new AnalogGyro(Ports.HORIZGYRO);

  private final BuiltInAccelerometer accel = new BuiltInAccelerometer();
  public final Output<Double> xAccel = addOutput("X accel", accel::getX);
  public final Output<Double> yAccel = addOutput("Y accel", accel::getY);
  public final Output<Double> zAccel = addOutput("Z accel", accel::getZ);

  public final Output<Double> gyroAngle = addOutput("gyroAngle", horizGyro::getAngle);
  public final Output<Integer> leftClicks = addOutput("leftClicks", encoderLeft::get);
  public final Output<Integer> rightClicks = addOutput("rightClicks", encoderRight::get);

  public final Output<Double> leftClickRate = addOutput("leftClickRate", encoderLeft::getRate);
  public final Output<Double> rightClickRate = addOutput("rightClickRate", encoderRight::getRate);

  public final Output<Double> leftDistance = addOutput("leftDistance", encoderLeft::getDistance);
  public final Output<Double> rightDistance = addOutput("rightDistance", encoderRight::getDistance);

  /* Auton Methods */
  public double getHeading() {
    var angle = -horizGyro.getAngle() * (Calibration.Drive.GYRO_REVERSED ? -1.0 : 1.0);
    return Math.IEEEremainder(angle, 360);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftClickRate.get(), rightClickRate.get());
  }

  public Pose2d getPose() {
    return m_driveOdometry.getPoseMeters();
  }

  public void updateOdometry() {
    m_driveOdometry.update(
        Rotation2d.fromDegrees(getHeading()),
        encoderLeft.getDistance(),
        encoderRight.getDistance());
  }

  public synchronized void resetSensors() {
    encoderLeft.reset();
    encoderRight.reset();
    horizGyro.reset();
  }

  /* Kinematics */
  public DifferentialDriveKinematics m_driveKinematics =
      new DifferentialDriveKinematics(Calibration.Drive.TRACK_WIDTH);

  public DifferentialDriveOdometry m_driveOdometry =
      new DifferentialDriveOdometry(m_driveKinematics, Rotation2d.fromDegrees(getHeading()));

  /* Drive Actions */

  public class Idle extends Action {
    public Idle() {
      super(Drive.this, Idle.class);
    }

    @Override
    public void run() {
      robotDrive.stopMotor();
    }
  }

  public final Action idle = new Idle();

  public class TankDrive extends Action {
    public final Input<Double> leftPower;
    public final Input<Double> rightPower;
    public final boolean squared;

    public TankDrive() {
      this(0, 0, true);
    }

    public TankDrive(boolean squared) {
      this(0, 0, squared);
    }

    public TankDrive(double defaultLeftPower, double defaultRightPower) {
      this(defaultLeftPower, defaultRightPower, true);
    }

    public TankDrive(double defaultLeftPower, double defaultRightPower, boolean squared) {
      super(Drive.this, TankDrive.class);
      leftPower = addInput("leftPower", defaultLeftPower, true);
      rightPower = addInput("rightPower", defaultRightPower, true);
      this.squared = squared;
    }

    @Override
    public void run() {
      if (leftPower.get() > 1
          || leftPower.get() < -1
          || rightPower.get() > 1
          || rightPower.get() < -1) {
        System.out.println("L" + leftPower.get() + "R" + rightPower.get());
      }
      robotDrive.tankDrive(leftPower.get(), rightPower.get(), squared);
    }
  }

  public class TankDriveVolts extends Action {
    public final Input<Double> leftVolts;
    public final Input<Double> rightVolts;

    public TankDriveVolts() {
      this(0, 0);
    }

    public TankDriveVolts(double defaultLeftVolts, double defaultRightVolts) {
      super(Drive.this, TankDriveVolts.class);
      leftVolts = addInput("leftVolts", defaultLeftVolts, true);
      rightVolts = addInput("rightVolts", defaultRightVolts, true);
    }

    @Override
    public void run() {
      m_left.setVoltage(leftVolts.get());
      m_right.setVoltage(-rightVolts.get());
    }
  }

  public class ArcadeDrive extends Action {
    public final Input<Double> movePower;
    public final Input<Double> rotatePower;
    public final boolean squared;

    public ArcadeDrive() {
      this(0, 0, true);
    }

    public ArcadeDrive(boolean squared) {
      this(0, 0, squared);
    }

    public ArcadeDrive(double defaultMovePower, double defaultRotPower) {
      this(defaultMovePower, defaultRotPower, true);
    }

    public ArcadeDrive(double defaultMovePower, double defaultRotPower, boolean squared) {
      super(Drive.this, ArcadeDrive.class);
      movePower = addInput("movePower", defaultMovePower, true);
      rotatePower = addInput("rotatePower", defaultRotPower, true);
      this.squared = squared;
    }

    @Override
    public void run() {
      robotDrive.arcadeDrive(movePower.get(), rotatePower.get(), squared);
    }
  }

  public Drive() {
    super(Drive.class);
    encoderLeft.setDistancePerPulse(Calibration.Drive.DISTANCE_PER_CLICK);
    encoderRight.setDistancePerPulse(Calibration.Drive.DISTANCE_PER_CLICK);
    robotDrive.setDeadband(0.04);
    setDefaultAction(idle);
  }
}
