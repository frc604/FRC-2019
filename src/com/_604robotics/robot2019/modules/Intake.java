package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Intake extends Module {
  public final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Ports.INTAKE_MOTOR);

  public Intake() {
    super(Intake.class);

    setDefaultAction(speed);
  }

  public class Idle extends Action {
    public Idle() {
      super(Intake.this, Idle.class);
    }

    @Override
    public void run() {
      intakeMotor.stopMotor();
    }
  }

  public final Idle idle = new Idle();

  public class Speed extends Action {
    private double speed;

    public Speed() {
      super(Intake.this, Speed.class);

      this.speed = 0;
    }

    public void run() {
      intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void set(double speed) {
      this.speed = speed;
    }
  }

  public final Speed speed = new Speed();
}
