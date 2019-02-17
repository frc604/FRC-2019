package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class HatchPusher extends Module {

    DoubleSolenoid pusher = new DoubleSolenoid(Ports.HATCH_PUSHER_A, Ports.HATCH_PUSHER_B);
    
    public Output<Boolean> pusherExtended;

    public HatchPusher() {
        super(HatchPusher.class);

        pusherExtended = addOutput("Pusher", () -> push.isRunning());

        setDefaultAction(pullBack);
    }

    public class Push extends Action {

        public Push() {
            super(HatchPusher.this, Push.class);
        }

        @Override
        public void run() {
            pusher.set(Value.kReverse);
        }


    }

    public class PullBack extends Action {

        public PullBack() {
            super(HatchPusher.this, PullBack.class);
        }

        @Override
        public void run() {
            pusher.set(Value.kForward);
        }
    }

    public Push push = new Push();
    public PullBack pullBack = new PullBack();

}