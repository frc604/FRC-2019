package com._604robotics.robotnik.prefabs.controller;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * <p>
 * Subclass of ProfiliedPIDController that has a feedforward for rotating arms.
 * </p>
 * This subclass requires an PIDSource as the PIDSource and uses only continuous
 * error. Zero is assumed to be horizontal. Users are responsible for properly
 * zeroing the PIDSource beforehand.
 */
public class ProfiliedRotatingArmPIDController extends ProfiledPIDController {
    private double encoderPeriod = 360;
    private double zeroOffset = 0;
    private double m_kf = 0.0;
    private double m_maxOutput = 1.0;
    private double m_minOutput = -1.0;

    public ProfiliedRotatingArmPIDController(double Kp, double Ki, double Kd, double Kf,
        TrapezoidProfile.Constraints constraints, double period) {
        super(Kp, Ki, Kd, constraints, period);
        this.m_kf = Kf;
    }

    public ProfiliedRotatingArmPIDController(double Kp, double Ki, double Kd, double Kf, TrapezoidProfile.Constraints constraints) {
        super(Kp, Ki, Kd, constraints, 0.02);
        this.m_kf = Kf;
    }

    public double getEncoderPeriod() {
        return encoderPeriod;
    }

    public void setEncoderPeriod(double encoderPeriod) {
        this.encoderPeriod = encoderPeriod;
    }

    public void setFeedforwardZeroOffset(double zeroOffset) {
        this.zeroOffset = zeroOffset;
    }

    public void setOutputRange(double min, double max) {
        this.m_minOutput = min;
        this.m_maxOutput = max;
    }

    /**
     * This is a physically based model which multiplies feed forward coefficient by cosine.
     * The feedforward calculates the expected torque needed to hold an arm steady, scaled to motor power.
     *
     *  @return the feed forward value
     */
    protected double calculateFeedForward(double measurment) {
        // Calculate cosine for torque factor
        double angle;
        double fValue;
        angle = measurment - zeroOffset;
        fValue = m_kf;

        // Cosine is periodic so sawtooth wraparound is not a concern
        angle/=encoderPeriod;
        angle*=(2*Math.PI);
        double cosine = Math.cos(angle);
        return fValue * cosine;
    }

    @Override
    public double calculate(double measurment, double setpoint) {
        if ( m_kf == 0.0 ) {
            return clamp(super.calculate(measurment, setpoint), m_minOutput, m_maxOutput);
        } else {
            return clamp(super.calculate(measurment, setpoint) + calculateFeedForward(measurment), m_minOutput, m_maxOutput);
        }
    }

    protected static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
      }
}
