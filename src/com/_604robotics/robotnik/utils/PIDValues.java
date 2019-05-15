package com._604robotics.robotnik.utils;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class PIDValues {
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    private double max;
    private double tolerance;
    private double period;

    private PIDSource source;
    private PIDOutput output;

    public PIDValues( double kP, double kI, double kD, double kF, double period ) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        this.period = period;
    }

    /**
     * Gets kP
     *
     * @return The value of kP
     */
    public double getkP() {
        return kP;
    }

    /**
     * Gets kI
     *
     * @return The value of kI
     */
    public double getkI() {
        return kI;
    }

    /**
     * Gets kD
     *
     * @return The value of kD
     */
    public double getkD() {
        return kD;
    }

    /**
     * Gets kF
     *
     * @return The value of kF
     */
    public double getkF() {
        return kF;
    }

    /**
     * Gets max
     *
     * @return The value of max
     */
    public double getMax() {
        return max;
    }

    /**
     * Gets tolerance
     *
     * @return The value of tolerance
     */
    public double getTolerance() {
        return tolerance;
    }

    /**
     * Gets period
     *
     * @return The value of period
     */
    public double getPeriod() {
        return period;
    }
}
