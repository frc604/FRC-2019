package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.controller.FlyWheelPDController;
import com._604robotics.robotnik.prefabs.devices.TalonPWMEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class Flywheel extends Module {
  private final FlyWheelPDController pid;

  private final WPI_TalonSRX flywheelMotor;
  private final WPI_TalonSRX transferMotor;
  private final WPI_TalonSRX feederMotor;

  private final TalonPWMEncoder flywheelEncoder;

  public Output<Double> pidError;
  public Output<Double> flywheelEncoderClicks;
  public Output<Double> flywheelEncoderRate;

  public Flywheel() {
    super(Flywheel.class);

    flywheelMotor = new WPI_TalonSRX(Ports.FLYWHEEL_MOTOR);
    transferMotor = new WPI_TalonSRX(Ports.TRANSFER_MOTOR);
    feederMotor = new WPI_TalonSRX(Ports.FEEDER_MOTOR);

    flywheelMotor.setInverted(true);
    transferMotor.setInverted(true);
    feederMotor.setInverted(true);

    flywheelMotor.configVoltageCompSaturation(12.0);
    flywheelMotor.enableVoltageCompensation(true);
    transferMotor.configVoltageCompSaturation(12.0);
    transferMotor.enableVoltageCompensation(true);
    feederMotor.configVoltageCompSaturation(12.0);
    feederMotor.enableVoltageCompensation(true);

    flywheelEncoder = new TalonPWMEncoder(flywheelMotor);
    flywheelEncoder.setInverted(true);

    flywheelEncoder.setDistancePerClick(0.1016 * Math.PI / 4096);

    flywheelEncoder.zero();

    flywheelEncoderClicks =
        addOutput("Flywheel Encoder Clicks", () -> flywheelEncoder.getPosition());
    flywheelEncoderClicks = addOutput("Flywheel Encoder Rate", () -> flywheelEncoder.getVelocity());

    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    this.pid =
        new FlyWheelPDController(
            Calibration.Flywheel.kP,
            Calibration.Flywheel.kD,
            feedforward,
            flywheelEncoder::getVelocity,
            this::setSpeed);

    setDefaultAction(stop);
  }

  public void setSpeed(double speed) {
    flywheelMotor.set(speed);
    transferMotor.set(speed * 0.6);
    feederMotor.set(speed * 0.6);
  }

  public class Setpoint extends Action {
    public Input<Double> setpoint;

    private Setpoint() {
      super(Flywheel.this, Setpoint.class);
      setpoint = addInput("Setpoint", 0.0, true);
    }

    @Override
    public void begin() {
      pid.setEnabled(true);
    }

    @Override
    public void run() {
      pid.setSetpoint(setpoint.get());
    }

    @Override
    public void end() {
      pid.setEnabled(false);
    }
  }

  public class Stop extends Action {

    private Stop() {
      super(Flywheel.this, Stop.class);
    }

    @Override
    public void begin() {
      pid.setEnabled(true);
    }

    @Override
    public void run() {
      pid.setSetpoint(0.0);
    }

    @Override
    public void end() {
      pid.setEnabled(false);
    }
  }

  public void resetEncoder() {
    flywheelEncoder.zero();
  }

  public Setpoint setpoint = new Setpoint();
  public Stop stop = new Stop();
}
