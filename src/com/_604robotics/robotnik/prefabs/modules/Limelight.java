package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robot2019.constants.Calibration;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;

import static com._604robotics.robot2019.constants.Calibration.*;

public class Limelight extends Module {

    private NetworkTable table;
    private HttpCamera cameraStream;

    public Output<Boolean> limelightHasTargets;
    public Output<Double> limelightX;
    public Output<Double> limelightY;
    public Output<Double> limelightArea;
    public Output<Double> limelightSkew;

    public Input<Integer> limelightLED; // TODO Find a way to store enum in networktables
    public Input<Integer> limelightStreamMode; // TODO Find a way to store enum in networktables
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

    /**
     * Creates a new Limelight module with the default NetworkTable name of "limelight"
     */
    public Limelight() {
        this("limelight");
    }

    /**
     * Creates a Limelight module with a custom NetworkTable table.
     * Allows for using multiple Limelights on one robot.
     * The name of the table must be changed in the settings of the Limelight camera too.
     * @param tableName Name of the table to access the Limelight at
     */
    public Limelight(String tableName) {
        super(Limelight.class);
        this.table = NetworkTableInstance.getDefault().getTable(tableName);

        limelightHasTargets = addOutput("limelightHasTargets", () -> table.getEntry("tv").getNumber((Number) 0).intValue() == 1);
        limelightX = addOutput("limelightX", () -> table.getEntry("tx").getDouble(0));
        limelightY = addOutput("limelightY", () -> table.getEntry("ty").getDouble(0));
        limelightArea = addOutput("limelightArea", () -> table.getEntry("ta").getDouble(0));
        limelightSkew = addOutput("limelightSkew", () -> table.getEntry("ts").getDouble(0));

        limelightPipeline = addInput("limelightPipelineInput", 0);
        limelightLED = addInput("limelightLEDInput", 0);
        limelightStreamMode = addInput("limelightStreamModeInput", 0);
        limelightSnapshotEnabled = addInput("limelightSnapshotEnabledInput", false);

        cameraStream = new HttpCamera("limelight", Calibration.LIMELIGHT_URL);
        cameraStream.setFPS(LIMELIGHT_FPS);
        cameraStream.setResolution(LIMELIGHT_RES_X, LIMELIGHT_RES_Y);

        setDefaultAction(driver);
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
                table.getEntry("ledMode").setNumber(limelightLED.get());
            }

            if( limelightSnapshotEnabled.isFresh() ) {
                table.getEntry("snapshot").setNumber((limelightSnapshotEnabled.get() ? 1 : 0));
            }

            if( limelightStreamMode.isFresh() ) {
                table.getEntry("stream").setNumber(limelightStreamMode.get());
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
            CameraServer.getInstance().addCamera(cameraStream);
        }

        @Override
        public void run() {
            if( limelightLED.isFresh() ) {
                table.getEntry("ledMode").setNumber(limelightLED.get());
            }

            if( limelightSnapshotEnabled.isFresh() ) {
                table.getEntry("snapshot").setNumber((limelightSnapshotEnabled.get() ? 1 : 0));
            }

            if( limelightStreamMode.isFresh() ) {
                table.getEntry("stream").setNumber(limelightStreamMode.get());
            }
        }

        @Override
        public void end() {
            CameraServer.getInstance().removeCamera("limelight");
        }
    }

    public final Action scan = new Scan();
    public final Action driver = new Driver();

    // Note: It is possible to use the raw contour data. This is not implemented here.

}
