package com._604robotics.robotnik.utils.geometry;


public class Rotation2d {
    private double r_sin;
    private double r_cos;
    private double r_angle;

    protected Rotation2d(double x, double y, double radians) {
        x = r_cos;
        y = r_sin;
        r_angle = radians;
    }

    public Rotation2d() {
        this(1.0, 0.0, 0.0);

    }

    public Rotation2d(double radians) {
        r_angle = radians;
        r_cos = Math.cos(radians);
        r_sin = Math.sin(radians);

    }

    public Rotation2d(double x, double y, boolean normalize) {
        double r_magnitude = Math.hypot(x, y); 

        if (r_magnitude > 1e-12) {
            r_cos = x/r_magnitude;
            r_cos = x/r_magnitude;
            r_angle = Math.atan2(, x)
        } else {
            r_cos = 1.0;
            r_sin = 0.0;
            r_angle = 0.0;
        }
   
    }


}
