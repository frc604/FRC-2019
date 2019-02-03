package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Intake extends Module {
    public final WPI_TalonSRX intake_motor = new WPI_TalonSRX(Ports.INTAKE_MOTOR);
    public final int default_speed = 0.01;

    public void setIntake (double speed) { 
        intake_motor.Set(speed);        
    }

    public void setOuttake (double speed) { 
        intake_motor.Set(speed * -1.0);        
    }

    public void end () {
        intake_motor.StopMotor();
    }
}