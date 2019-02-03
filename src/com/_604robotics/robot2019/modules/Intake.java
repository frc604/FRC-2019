package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Intake extends Module {
    public final TalonSRX intake_motor = new TalonSRX(Ports.INTAKE_MOTOR);
    public final double default_speed = 0.01;

    public void setIntake (double speed) { 
        intake_motor.set(ControlMode.PercentOutput, speed);       
    }

    public void setOuttake (double speed) { 
        intake_motor.set(ControlMode.PercentOutput, -speed);       
    }

    public void end () {
        intake_motor.set(ControlMode.PercentOutput, 0.0);    
    }

    public Intake() {
        super(Intake.class);
    }
}