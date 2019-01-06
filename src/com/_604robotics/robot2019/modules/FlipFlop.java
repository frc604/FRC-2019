package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class FlipFlop extends Module {
	/*private final MultiOutput motors = new MultiOutput(
            new Victor(Ports.INTAKE_LEFT_MOTOR),
            new Victor(Ports.INTAKE_RIGHT_MOTOR){{setInverted(true);}});
	*/
    private final DoubleSolenoid piston =
            new DoubleSolenoid(Ports.FLIP_FLOP_EXTEND_SOLENOID, Ports.FLIP_FLOP_RETRACT_SOLENOID);

    public final Input<Double> transitionTime = addInput("transitionTime", Calibration.FLIP_FLOP_TRANSITION_TIME);

    private boolean startup = true;

    public class Retract extends Action {
        private final SmartTimer timer = new SmartTimer();

        public final Output<Boolean> completed =
                addOutput("completed", () -> startup || timer.get() >= transitionTime.get());

        private Retract () {
            super(FlipFlop.this, Retract.class);
        }

        @Override
        protected void begin () {
            timer.start();
            piston.set(DoubleSolenoid.Value.kReverse);
        }/*

        @Override
        protected void run() {
        	motors.stopMotor();
        }*/
        
        @Override
        protected void end () {
            timer.stopAndReset();
            startup = false;
        }
    }

    public final Retract retract = new Retract();

    public class Extend extends Action {
        private final SmartTimer timer = new SmartTimer();

        public final Output<Boolean> completed = addOutput("completed", () -> timer.get() >= transitionTime.get());

        private Extend () {
            super(FlipFlop.this, Extend.class);
        }

        @Override
        protected void begin () {
            timer.start();
            piston.set(DoubleSolenoid.Value.kForward);
        }
/*
        @Override
        protected void run () {
        	robo
        }*/

        @Override
        protected void end () {
            timer.stopAndReset();
        }
    }

    public final Extend extend = new Extend();

    public FlipFlop () {
        super(FlipFlop.class);
        setDefaultAction(retract);
    }

    @Override
    protected void begin () {
        startup = true;
    }
}