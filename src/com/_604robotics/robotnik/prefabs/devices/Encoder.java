package com._604robotics.robotnik.prefabs.devices;

import edu.wpi.first.wpilibj.PIDSource;

public interface Encoder extends PIDSource {
    double getValue();
}
