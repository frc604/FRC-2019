package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Intake extends Module {
    public Output<Boolean> LeftLimitSwitch;

    public final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Ports.INTAKE_MOTOR);
    private final DigitalInput ballSwitch = new DigitalInput(Ports.INTAKE_LIMIT);

    public class Idle extends Action {
        public Idle () {
            super(Intake.this, Idle.class);
        }

        @Override
        public void run () {
            intakeMotor.stopMotor();
        }

        public boolean getState() {
            return ballSwitch.get();
        }
    }

    public final Action idle = new Idle();

    public class Speed extends Action {
        public void set(double speed) {
            System.out.println("SPEEDDD333333");
            intakeMotor.set(ControlMode.PercentOutput, speed);
        }

        public boolean getState() {
            return ballSwitch.get();
        }

        public Speed() {
            super(Intake.this, Speed.class);
        }
    }
	
	public final Action speed = new Speed();

    public Intake() {
        super(Intake.class);
        LeftLimitSwitch = addOutput("Left Limit Switch", () -> ballSwitch.get());

        setDefaultAction(speed);
    }
}