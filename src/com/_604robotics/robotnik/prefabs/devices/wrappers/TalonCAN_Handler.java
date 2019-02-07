package com._604robotics.robotnik.prefabs.devices.wrappers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonCAN_Handler extends WPI_TalonSRX{

    double prev_value = super.get();
    ControlMode prev_mode = super.getControlMode();

    public double getPrev() {
        return prev_value;
    }

    @Override
    public void set(ControlMode mode, double speed) {
        if ( prev_value != speed || prev_mode != mode ) {
            super.set(mode, speed);
            prev_value = speed;
            prev_mode = mode;
        }
    }

    public TalonCAN_Handler(int port_SRX) {
        super(port_SRX);
    }

}