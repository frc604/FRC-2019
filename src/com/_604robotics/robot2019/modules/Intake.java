package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Intake extends Module {
    public Output<Boolean> holdingBall;

    public final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Ports.INTAKE_MOTOR);
    private final DigitalInput ballSwitch = new DigitalInput(Ports.INTAKE_LIMIT);
    
    public Intake() {
        super(Intake.class);
        
        holdingBall = addOutput("Ball in Intake", () -> ballSwitch.get());

        setDefaultAction(speed);
    }

    public class Idle extends Action {
        public Idle () {
            super(Intake.this, Idle.class);
        }

        @Override
        public void run() {
            intakeMotor.stopMotor();
        }
    }

    public final Idle idle = new Idle();

    public class Speed extends Action {
        public Speed() {
            super(Intake.this, Speed.class);
        }
        
        public void set(double speed) {
            intakeMotor.set(ControlMode.PercentOutput, speed);
        }
    }
	
	public final Speed speed = new Speed();
}