package com._604robotics.robotnik.prefabs.inputcontroller.xbox;

import com._604robotics.robotnik.prefabs.inputcontroller.ControllerRumble;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A Xbox controller.
 */
public class XboxController {
    /**
     * The controller's buttons.
     */
    public final XboxControllerButtons buttons;

    /**
     * The controller's left stick.
     */
    public final XboxControllerStick leftStick;

    /**
     * The controller's right stick.
     */
    public final XboxControllerStick rightStick;

    /**
     * The controller's triggers.
     */
    public final XboxControllerTriggers triggers;

    /**
     * The controller's D-pad.
     */
    public final XboxControllerDpad dpad;

    /**
     * The controller's rumble feature.
     */
    public final ControllerRumble rumble;

    /**
     * Creates an Xbox controller.
     * @param port Port of the controller.
     */
    public XboxController (int port) {
        this(new Joystick(port));
    }

    /**
     * Creates an Xbox controller.
     * @param joystick Underlying joystick object to wrap.
     */
    public XboxController (Joystick joystick) {
        this.buttons = new XboxControllerButtons(joystick);

        this.leftStick  = new XboxControllerStick(joystick, 0, 1);
        this.rightStick = new XboxControllerStick(joystick, 4, 5);

        this.triggers = new XboxControllerTriggers(joystick, 2, 3);

        this.dpad = new XboxControllerDpad(joystick);

        this.rumble = new ControllerRumble(joystick);
    }
}