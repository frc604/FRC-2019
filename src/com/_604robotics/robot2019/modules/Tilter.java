package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com._604robotics.robotnik.prefabs.devices.wrappers.TalonCAN_Handler;
import edu.wpi.first.wpilibj.DigitalInput;

public class Tilter extends Module {
    private TalonCAN_Handler tiltMotor;
    private DigitalInput tiltComplete;

    public Output<Boolean> isTilted;

    public Tilter() {
        super(Tilter.class);

        tiltMotor = new TalonCAN_Handler(Ports.TILT_MOTOR);
        tiltComplete = new DigitalInput(Ports.TILT_SWITCH);

        isTilted = addOutput("Tilted", () -> tilt.isRunning());

        setDefaultAction(stow);
    }

    public class Stow extends Action {

        public Stow() {
            super(Tilter.this, Stow.class);
        }

        @Override
        public void begin() {
            tiltMotor.set(ControlMode.PercentOutput, 0);
        }

    }

    public Action stow = new Stow();

    public class Tilt extends Action {

        public Tilt() {
            super(Tilter.this, Tilt.class);
        }

        @Override
        public void run() {
            if( !tiltComplete.get() ) {
                tiltMotor.set(ControlMode.PercentOutput, 1);
            } else {
                tiltMotor.set(ControlMode.PercentOutput, 0);
            }
        }
    }

    public Action tilt = new Tilt();
}
