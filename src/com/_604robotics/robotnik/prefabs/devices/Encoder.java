package com._604robotics.robotnik.prefabs.devices;

import edu.wpi.first.wpilibj.PIDSource;

public interface Encoder extends PIDSource {
    // FIXME Check if CounterBase can replace this, because the wpilib encoder can't extend this class
    double getValue(); // TODO should this be an int? Probably. See TalonPWMEncoder for problems
    void zero();
}
