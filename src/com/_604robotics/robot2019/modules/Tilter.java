package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

public class Tilter extends Module {
    private WPI_TalonSRX tiltMotor;
    private DigitalInput tiltComplete;

    public Output<Boolean> isTilted;

    public Tilter() {
        super(Tilter.class);

        tiltMotor = new WPI_TalonSRX(Ports.TILT_MOTOR);
        //tiltComplete = new DigitalInput(Ports.TILT_SWITCH);

        isTilted = addOutput("Tilted", () -> tilt.isRunning());

        setDefaultAction(stow);
    }

    public class Stow extends Action {

        public Stow() {
            super(Tilter.this, Stow.class);
        }

        @Override
        public void begin() {
            tiltMotor.set(ControlMode.PercentOutput, 0.0);
            tiltMotor.stopMotor();
        }

    }

    public Action stow = new Stow();

    public class Tilt extends Action {

        public Tilt() {
            super(Tilter.this, Tilt.class);
        }

        @Override
        public void run() {
            tiltMotor.set(ControlMode.PercentOutput, 1);
        }
    }

    public class Retract extends Action {
        public Retract() {
            super(Tilter.this, Retract.class);
        }

        @Override
        public void run() {
            tiltMotor.set(ControlMode.PercentOutput, -0.3);
        }
    }
    public Action tilt = new Tilt();
    public Action retract = new Retract();
}
