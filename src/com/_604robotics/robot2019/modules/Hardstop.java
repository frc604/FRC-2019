package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Hardstop extends Module {
    private final DoubleSolenoid hardstop;

    public Output<Boolean> closed;

    public Hardstop() {
        super(Hardstop.class);

        hardstop = new DoubleSolenoid(Ports.HARDSTOP_A, Ports.HARDSTOP_B);
		
        open = new Open();
        close = new Close();

        closed = addOutput("closed", () -> close.isRunning());
		
        setDefaultAction(close);
    }

    public class Open extends Action {
        public Open() {
            super(Hardstop.this, Open.class);
        }
		
		@Override
		public void run() {
			hardstop.set(Value.kForward);
		}
    }

    public class Close extends Action {
        public Close() {
            super(Hardstop.this, Close.class);
        }
		
		@Override
		public void run() {
			hardstop.set(Value.kReverse);
		}
    }

    public Open open;
    public Close close;
}
