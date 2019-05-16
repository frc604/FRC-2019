package com._604robotics.robotnik.prefabs.devices;

import com._604robotics.robotnik.prefabs.controller.ExtendablePIDController;
import com._604robotics.robotnik.prefabs.devices.wrappers.RampMotor;
import com._604robotics.robotnik.utils.PIDValues;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * Differential serve drive module, where by running two motors with each other, the module drives. By driving the two
 * motors against each other, the module turns. This allows for higher pushing power, if you can avoid browning out.
 *
 * <p>This module also assumes that a 0 degree turn is the same as a 180 degree turn, and will run the motors backwards
 * as a method to more quickly align.</p>
 *
 * Some basic vector math and power scaling allows for turning while driving.
 *
 * The advantage of this is the additional power that can be utilized when driving and *not* spinning in circles.
 * Imagine: a swerve drive beating a tank drive because of having two motors per wheel. Of course, this is not close
 * to being garunteed, due to having only four wheels total, but it is nice to think about.
 */
public class DifferentialSwerveUnit extends SwerveUnit {
    private final RampMotor drive;
    private final RampMotor turn;

    private final ExtendablePIDController turnPID;

    private final Encoder driveEncoder;
    private final Encoder turnEncoder;

    private final double clicksPerRotation;

    public DifferentialSwerveUnit(RampMotor drive, RampMotor turn, Encoder driveEncoder, Encoder turnEncoder,
                                  PIDValues turnPIDValues, double clicksPerRotation) {
        this.drive = drive;
        this.turn = turn;

        this.driveEncoder = driveEncoder;
        this.turnEncoder = turnEncoder;

        this.turnPID = new ExtendablePIDController(turnPIDValues.getkP(), turnPIDValues.getkI(), turnPIDValues.getkD(),
                turnPIDValues.getkF(), turnEncoder, turn);
        this.turnPID.setEnabled(false);

        this.clicksPerRotation = clicksPerRotation;
    }

    public DifferentialSwerveUnit(SpeedController drive, SpeedController turn, Encoder driveEncoder, Encoder turnEncoder,
                                  PIDValues turnPIDValues, double clicksPerRotation) {
        this(new RampMotor(drive), new RampMotor(turn), driveEncoder, turnEncoder, turnPIDValues, clicksPerRotation);
    }

    @Override
    public void setAngle(double angle) {
        // TODO convert angle to clicks

        turnPID.setEnabled(true);
        turnPID.setSetpoint(angle);

        feed();
    }

    @Override
    public void setDrive(double power) {
        drive.set(power);

        feed();
    }

    @Override
    public double getDrive() {
        return drive.get();
    }

    @Override
    public double getAngle() {
        return turnPID.getSetpoint();
    }

    @Override
    public void stopMotor() {
        drive.stopMotor();
        turnPID.setEnabled(false);
        turn.stopMotor();

        feed();
    }

    @Override
    public String getDescription() {
        return "ServeUnit";
    }

    @Override
    public double getTurnClicks() {
        return turnEncoder.getValue();
    }

    @Override
    public double getDriveClicks() {
        return driveEncoder.getValue();
    }

    @Override
    public synchronized void resetDriveEncoder() {
        driveEncoder.zero();
    }

    @Override
    public synchronized void resetTurnEncoder() {
        turnEncoder.zero();
    }


    // These two outputs are needed to prevent
    private class TurnMotor implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            turn.set(output);
        }

        void setClicks(double clicks) {

        }
    }

    private class DriveMotor implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            drive.set(output);
        }

        void setPower(double power) {

        }
    }

}
