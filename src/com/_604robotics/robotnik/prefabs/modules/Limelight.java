package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends Module {

    /*
     * Perhaps refactor so that the targets are saved to Output<> tables
     * and accessed that way, and checked using a run() function
     *
     * That sounds good.
     */

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
        super(Limelight.class);
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        setDefaultAction(scan);
    }

    private class Scan extends Action {
        public Scan() {
            super(Limelight.this, Scan.class);
        }

        @Override
        public void begin() {
            table.getEntry("camMode").setNumber(0);
        }
    }

    private class Driver extends Action {
        public Driver() {
            super(Limelight.this, Driver.class);
        }

        @Override
        public void begin() {
            // When swapping to this action, we need to disable vision processing
            table.getEntry("camMode").setNumber(1);
        }
    }

    public final Action scan = new Scan();
    public final Action driver = new Driver();

    public boolean hasTarget() {
        return this.getRunningAction() == scan && table.getEntry("tv").getNumber((Number) 0).intValue() == 1;
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
