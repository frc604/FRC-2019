package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import edu.wpi.first.wpilibj.Relay;

public class SignalLight extends Module {
    private final Relay light = new Relay(Ports.SIGNAL_LIGHT_RELAY);

    private class Switch extends Action {
        private final Relay.Value value;

        public Switch (String name, Relay.Value value) {
            super(SignalLight.this, name);
            this.value = value;
        }

        @Override
        protected void run () {
            light.set(value);
        }
    }

    public final Action off = new Switch("Off", Relay.Value.kOff);
    public final Action on = new Switch("On", Relay.Value.kForward);

    public SignalLight () {
        super(SignalLight.class);
        setDefaultAction(off);
    }
}