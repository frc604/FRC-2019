package com._604robotics.robot2019.modules;

import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;


public class BenchTalon extends Module {
    private WPI_TalonSRX benchMotor;

    public BenchTalon(Integer DeviceID, Integer ControlFramePeriod, Integer Status1FramePeriod, Integer Status2FramePeriod) {
        super(BenchTalon.class);

        benchMotor = new WPI_TalonSRX(DeviceID);
        benchMotor.setControlFramePeriod(ControlFrame.Control_3_General, ControlFramePeriod);
        benchMotor.setStatusFramePeriod(StatusFrame.Status_1_General , Status1FramePeriod);
        benchMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0 , Status2FramePeriod);
    }

    public void move(Double power) {
        benchMotor.set(ControlMode.PercentOutput, power);

    }

}
