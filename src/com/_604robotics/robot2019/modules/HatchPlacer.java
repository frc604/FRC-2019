package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class HatchPlacer extends Module {
    private DoubleSolenoid placer;
    private DigitalInput leftSwitch;
    private DigitalInput rightSwitch;

    public Output<Boolean> isHolding;
    public Output<Boolean> aligned;

    public HatchPlacer() {
        super(HatchPlacer.class);

        placer = new DoubleSolenoid(Ports.HATCH_PLACER_A, Ports.HATCH_PLACER_B);
        leftSwitch = new DigitalInput(Ports.HATCH_LEFT_SWITCH);
        rightSwitch = new DigitalInput(Ports.HATCH_RIGHT_SWITCH);

        isHolding = addOutput("Holding", () -> hold.isRunning());
        aligned = addOutput("Hatch Aligned", () -> leftSwitch.get() && rightSwitch.get());

        setDefaultAction(hold); // TODO make dashboard value
    }

    public class Release extends Action {
        public Release() {
            super(HatchPlacer.this, Release.class);

            placer.set(DoubleSolenoid.Value.kForward);
        }
    }

    public class Hold extends Action {
        public Hold() {
            super(HatchPlacer.this, Hold.class);

            placer.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public Release release = new Release();
    public Hold hold = new Hold();
}
