package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends Module {

    private NetworkTable table;

    public Output<Boolean> limelightHasTargets;
    public Output<Double> limelightX;
    public Output<Double> limelightY;
    public Output<Double> limelightArea;
    public Output<Double> limelightSkew;

    public Input<LEDState> limelightLED;
    public Input<StreamMode> limelightStreamMode;
    public Input<Integer> limelightPipeline;
    public Input<Boolean> limelightSnapshotEnabled;

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

        limelightHasTargets = addOutput("limelightHasTargets", () -> table.getEntry("tv").getNumber((Number) 0).intValue() == 1);
        limelightX = addOutput("limelightX", () -> table.getEntry("tx").getDouble(0));
        limelightY = addOutput("limelightY", () -> table.getEntry("ty").getDouble(0));
        limelightArea = addOutput("limelightArea", () -> table.getEntry("ta").getDouble(0));
        limelightSkew = addOutput("limelightSkew", () -> table.getEntry("ts").getDouble(0));

        limelightPipeline = addInput("limelightPipelineInput", 0);
        limelightLED = addInput("limelightLEDInput", LEDState.CURRENT);
        limelightStreamMode = addInput("limelightStreamModeInput", StreamMode.STANDARD);
        limelightSnapshotEnabled = addInput("limelightSnapshotEnabledInput", false);

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

        @Override
        public void run() {
            if( limelightPipeline.isFresh() ) {
                table.getEntry("pipeline").setNumber(limelightPipeline.get());
            }

            if( limelightLED.isFresh() ) {
                table.getEntry("ledMode").setNumber(limelightLED.get().ordinal());
            }

            if( limelightSnapshotEnabled.isFresh() ) {
                table.getEntry("snapshot").setNumber((limelightSnapshotEnabled.get() ? 1 : 0));
            }

            if( limelightStreamMode.isFresh() ) {
                table.getEntry("stream").setNumber(limelightStreamMode.get().ordinal());
            }
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

        @Override
        public void run() {
            if( limelightLED.isFresh() ) {
                table.getEntry("ledMode").setNumber(limelightLED.get().ordinal());
            }

            if( limelightSnapshotEnabled.isFresh() ) {
                table.getEntry("snapshot").setNumber((limelightSnapshotEnabled.get() ? 1 : 0));
            }

            if( limelightStreamMode.isFresh() ) {
                table.getEntry("stream").setNumber(limelightStreamMode.get().ordinal());
            }
        }
    }

    public final Action scan = new Scan();
    public final Action driver = new Driver();

    // Note: It is possible to use the raw contour data. This is not implemented here.

}
