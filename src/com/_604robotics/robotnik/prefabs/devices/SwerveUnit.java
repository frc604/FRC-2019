package com._604robotics.robotnik.prefabs.devices;

import edu.wpi.first.wpilibj.MotorSafety;

/**
 * Represents a single swerve module on the robot. Implementation is likely to involve a turning motor, and a drive
 * motor. Mechanical differences may occur, necessitating the creation of this class as a common parent.
 *
 * <p>Extends {@link MotorSafety} to force proper handling of drive motors, which are dangerous when out of control.</p>
 */
public abstract class SwerveUnit extends MotorSafety {
    // TODO Add javadocs

    /**
     * Sets the orientation of the module.
     * @param angle Desired angle of the module, assuming clockwise is positive
     */
    public abstract void setAngle(double angle);
    public abstract void setDrive(double power);

    public abstract double getDrive();
    public abstract double getAngle();

    public abstract double getDriveClicks();
    public abstract double getTurnClicks();

    public abstract void resetDriveEncoder();
    public abstract void resetTurnEncoder();
}
