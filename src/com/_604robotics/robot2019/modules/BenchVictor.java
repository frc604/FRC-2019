package com._604robotics.robot2019.modules;

import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;


public class BenchVictor extends Module {
    private WPI_VictorSPX benchMotor;

    public BenchVictor(Integer DeviceID, Integer FramePeriod) {
        super(BenchVictor.class);

        benchMotor = new WPI_VictorSPX(DeviceID);
        benchMotor.setControlFramePeriod(ControlFrame.Control_3_General, FramePeriod);
    }

    public void move(Double power) {
        benchMotor.set(ControlMode.PercentOutput, power);

    }

}
