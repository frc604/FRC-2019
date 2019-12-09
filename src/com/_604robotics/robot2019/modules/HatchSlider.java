package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class HatchSlider extends Module {
  /*
   * Driven forward / back with two pistons
   * Slides left/right passively
   * Detect centered with two limit switches
   * One piston to release hatch
   */
  DoubleSolenoid left;

  public Output<Boolean> isForward;

  public HatchSlider() {
    super(HatchSlider.class);

    left = new DoubleSolenoid(Ports.HATCH_LEFT_SLIDE_A, Ports.HATCH_LEFT_SLIDE_B);

    isForward = addOutput("Forward", () -> front.isRunning());

    setDefaultAction(back);
  }

  public class Back extends Action {
    public Back() {
      super(HatchSlider.this, Back.class);
    }

    @Override
    public void run() {
      left.set(Value.kReverse);
    }
  }

  public class Front extends Action {
    public Front() {
      super(HatchSlider.this, Front.class);
    }

    @Override
    public void run() {
      left.set(Value.kForward);
    }
  }

  public Back back = new Back();
  public Front front = new Front();
}
