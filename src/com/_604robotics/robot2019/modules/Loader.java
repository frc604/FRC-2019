package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.prefabs.devices.wrappers.MultiOutput;

import edu.wpi.first.wpilibj.Victor;

public class Loader extends Module {
	
	private final MultiOutput wheel_bot = new MultiOutput(new Victor(Ports.WHEEL_BOT));
	private final MultiOutput belt = new MultiOutput(new Victor(Ports.BELT));
	
	private final Action idle = new Idle();
	private final Action load = new Load();
	
	public class Idle extends Action {
        public Idle () {
            super(Loader.this, Idle.class);
        }

        @Override
        public void run () {
        	wheel_bot.stopMotor();
        	belt.stopMotor();
        }
    }
	
	public class Load extends Action {
		public final Input<Boolean> on;
		
		public Load() {
			this(false);
		}
		public Load(boolean defaultOn) {
			super(Loader.this, Load.class);
			this.on = addInput("Loading", defaultOn, true);
		}
		
		@Override
		public void run() {
			if( on.get() ) {
				wheel_bot.set(Calibration.WHEEL_BOT_SPEED);
				belt.set(Calibration.BELT_SPEED);
			}
		}
		
		@Override
		public void end() {
			wheel_bot.stopMotor();
			belt.stopMotor();
		}
	}
	
	public Loader() {
		super(Loader.class);
		setDefaultAction(idle);
	}
}