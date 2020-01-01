package com._604robotics.robotnik.prefabs.coordinators;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;

/**
 * A class that allows a runnable to be run as a coordinator. It can be used to inline macros in
 * autonomus code.
 */
public class ActionCoordinator extends Coordinator {
  private Runnable action;
  private Type type;

  private SmartTimer timer = new SmartTimer();

  private double time;

  /**
   * Creates an Action Coordinator that runs the runnable for the specfied ammount of time.
   *
   * @param action Runnable to run.
   * @param time Time to run runnable for.
   */
  public ActionCoordinator(Runnable action, double time) {
    this.action = requireNonNullParam(action, "action", "ActionCoordinator");

    if (time <= 0) {
      throw new IllegalArgumentException("Must run for a positive amount of time!");
    }

    this.time = time;

    type = Type.TIMED;
  }

  /**
   * Creates an Action Coordinator that runs the runnable once.
   *
   * @param action Runnable to run.
   */
  public ActionCoordinator(Runnable action) {
    this.action = requireNonNullParam(action, "action", "ActionCoordinator");

    type = Type.INSTANT;
  }

  /** Creates an empty Action Coordinator that runs nothing. Useful for debugging. */
  public ActionCoordinator() {
    type = Type.EMPTY;
  }

  @Override
  protected void begin() {
    switch (type) {
      case TIMED:
        timer.start();
        break;
      case INSTANT:
        action.run();
        break;
      case EMPTY:
        break;
    }
  }

  @Override
  protected boolean run() {
    boolean running = false;

    switch (type) {
      case TIMED:
        action.run();
        running = !timer.hasReachedTime(time);
        break;
      case INSTANT:
        break;
      case EMPTY:
        break;
    }

    return running;
  }

  public enum Type {
    TIMED,
    INSTANT,
    EMPTY
  }
}
