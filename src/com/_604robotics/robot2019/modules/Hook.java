package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Hook extends Module {
  private final DoubleSolenoid hook;

  public Output<Boolean> isHolding;
  public Output<Boolean> isHoldingDash;
  public Output<Boolean> aligned;

  public Hook() {
    super(Hook.class);

    hook = new DoubleSolenoid(Ports.HOOK_A, Ports.HOOK_B);

    release = new Release();
    hold = new Hold();

    isHolding = addOutput("Holding", () -> hold.isRunning());
    aligned = addOutput("Hatch Aligned", () -> false);

    setDefaultAction(hold);
  }

  public class Release extends Action {
    public Release() {
      super(Hook.this, Release.class);
    }

    @Override
    public void run() {
      hook.set(Value.kReverse);
    }
  }

  public class Hold extends Action {
    public Hold() {
      super(Hook.this, Hold.class);
    }

    @Override
    public void run() {
      hook.set(Value.kForward);
    }
  }

  public Release release;
  public Hold hold;
}
