package com._604robotics.robotnik.prefabs.devices;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    private NetworkTable table;

    public enum LEDState {
        CURRENT,
        OFF,
        BLINK,
        ON
    }

    public enum StreamMode {
        STANDARD,
        PIPMAIN,
        PIPSECONDARY
    }

    public Limelight() {
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getNumber((Number) 0).intValue() == 1;
    }

    public Double getArea() {
        return table.getEntry("ta").getDouble(0);
    }

    public Double getVirtOffset() {
        return table.getEntry("ty").getDouble(0);
    }

    public Double getHorizOffset() {
        return table.getEntry("tx").getDouble(0);
    }

    public Double getAngle() {
        return table.getEntry("ts").getDouble(0);
    }

    public void setLED(LEDState state) {
        table.getEntry("ledMode").setNumber(state.ordinal());
    }

    public void processVision(boolean enabled) {
        table.getEntry("camMode").setNumber((enabled ? 1 : 0));
    }

    public void setPipeline(int pipe) {
        table.getEntry("pipeline").setNumber(pipe);
    }

    public void setStreamMode(StreamMode stream) {
        table.getEntry("stream").setNumber(stream.ordinal());
    }

    public void takeSnapshots(boolean enabled) {
        table.getEntry("snapshot").setNumber((enabled ? 1 : 0));
    }

    // Note: It is possible to use the raw contour data. This is not implemented here.

}
