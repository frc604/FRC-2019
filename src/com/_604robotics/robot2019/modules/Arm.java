package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.controller.ProfiledRotatingArmPIDController;
import com._604robotics.robotnik.prefabs.devices.TalonPWMEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class Arm extends Module {
  private ProfiledRotatingArmPIDController pid;

  private WPI_TalonSRX leftMotor;
  private WPI_TalonSRX rightMotor;

  private TalonPWMEncoder leftEncoder;

  public Input<Double> holdPoint;

  public Output<Double> pidError;
  public Output<Double> rightEncoderClicks;
  public Output<Double> leftEncoderClicks;
  public Output<Double> redundantEncoderClicks;

  public Arm() {
    super(Arm.class);

    leftMotor = new WPI_TalonSRX(Ports.ARM_LEFT_MOTOR);
    rightMotor = new WPI_TalonSRX(Ports.ARM_RIGHT_MOTOR);

    leftMotor.setInverted(true);

    rightMotor.set(ControlMode.Follower, Ports.ARM_LEFT_MOTOR);
    rightMotor.setInverted(InvertType.OpposeMaster);

    leftMotor.configVoltageCompSaturation(12.0);
    leftMotor.enableVoltageCompensation(true);

    leftEncoder = new TalonPWMEncoder(rightMotor);
    leftEncoder.setInverted(false);

    leftEncoder.zero();

    holdPoint = addInput("Setpoint", 300.0);
    leftEncoderClicks = addOutput("Left Encoder Clicks", () -> leftEncoder.getPosition());

    ArmFeedforward feedforward = new ArmFeedforward(0, Calibration.Arm.kF, 0, 0);

    this.pid =
        new ProfiledRotatingArmPIDController(
            Calibration.Arm.kP,
            Calibration.Arm.kI,
            Calibration.Arm.kD,
            feedforward,
            new TrapezoidProfile.Constraints(10000, 3000),
            leftEncoder::getPosition,
            leftMotor::set);

    pid.setOutputRange(-0.65, 0.65);
    pid.setEncoderPeriod(Calibration.Arm.CLICKS_FULL_ROTATION);
    pid.setFeedforwardZeroOffset(Calibration.Arm.HORIZONTAL_POSITION);
    setDefaultAction(move);
  }

  public class Hold extends Action {

    private Hold() {
      super(Arm.this, Hold.class);
      holdPoint = addInput("Hold Point", 300.0, true);
    }

    @Override
    public void begin() {
      pid.setEnabled(true);
    }

    @Override
    public void run() {
      pid.setGoal(holdPoint.get());
    }

    @Override
    public void end() {
      pid.setEnabled(false);
    }
  }

  public class Move extends Action {
    public Input<Double> inputPower;

    private Move() {
      super(Arm.this, Move.class);
      inputPower = addInput("Input Power", 0.0, true);
    }

    @Override
    public void begin() {
      pid.setEnabled(false);
    }

    @Override
    public void run() {
      leftMotor.set(inputPower.get());
    }
  }

  public class Setpoint extends Action {
    public Input<Double> setpoint;

    private Setpoint() {
      super(Arm.this, Setpoint.class);
      setpoint = addInput("Setpoint", 0.0, true);
    }

    @Override
    public void begin() {
      pid.setEnabled(true);
    }

    @Override
    public void run() {
      pid.setGoal(setpoint.get());
    }

    @Override
    public void end() {
      pid.setEnabled(false);
    }
  }

  public void resetEncoder() {
    leftEncoder.zero();
  }

  public Hold hold = new Hold();
  public Move move = new Move();
  public Setpoint setpoint = new Setpoint();
}
