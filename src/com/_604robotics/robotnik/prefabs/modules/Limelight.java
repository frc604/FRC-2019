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
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

import static com._604robotics.robot2019.constants.Calibration.*;

public class Limelight extends Module {

    private NetworkTable table;
    private HttpCamera cameraStream;

    public Output<Boolean> limelightHasTargets;
    public Output<Double> limelightX;
    public Output<Double> limelightY;
    public Output<Double> limelightArea;
    public Output<Double> limelightSkew;

    public Output<Double> limelightDistance;

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

        limelightDistance = addOutput("limelightDistance", this::getDistance);

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

    private double getDistance() {
        return (Calibration.LIMELIGHT_HEIGHT - Calibration.TARGET_HEIGHT) *
            Math.tan(this.limelightY.get() + Calibration.LIMELIGHT_ANGLE);
    }

    // Note: It is possible to use the raw contour data. This is not implemented here.

    public static class HorizontalError implements PIDSource {
        private Limelight limelight;
        private double setpoint;

        public HorizontalError( Limelight limelight ) {
            this(limelight, 0);
        }

        /**
         * Creates a new PID representing horizontal angle error of the limelight
         * @param limelight Limelight object to use
         * @param setpoint The angle offset to use
         */
        public HorizontalError( Limelight limelight, double setpoint ) {
            this.limelight = limelight;
            this.setpoint = setpoint;
        }

        /**
         * Sets the angle offset to use
         * @param setpoint Angle offset to use
         */
        public void setSetPoint( double setpoint ) {
            this.setpoint = setpoint;
        }

        @Override
        public void setPIDSourceType( PIDSourceType pidSource ) {
            if( pidSource != PIDSourceType.kDisplacement ) {
                throw new IllegalArgumentException("Limelight PID only accepts Displacement source type");
            }
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }

        @Override
        public double pidGet() {
            // I have been very silly. Limelight calculates angle for you.
            return limelight.limelightX.get() - setpoint;
        }
    }

    public static class VerticalError implements PIDSource {
        private Limelight limelight;
        private double setpoint;

        public VerticalError( Limelight limelight ) {
            this(limelight, 0);
        }

        public VerticalError( Limelight limelight, double setpoint ) {
            this.limelight = limelight;
            this.setpoint = setpoint;
        }

        public void setSetpoint( double setpoint ) {
            this.setpoint = setpoint;
        }

        @Override
        public void setPIDSourceType( PIDSourceType pidSource ) {
            if( pidSource != PIDSourceType.kDisplacement ) {
                throw new IllegalArgumentException("Limelight PID only accepts Displacement source type");
            }

        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }

        @Override
        public double pidGet() {
            return limelight.limelightX.get() - setpoint;
        }
    }

    public static class DistanceError implements PIDSource {
        private Limelight limelight;
        private double distance;

        /**
         * Creates a new PIDSource representing the distance from a calibrated target
         * using the angle of the limelight relative to the target
         * @param limelight The limelight object representing the limelight
         * @param distance Goal distance the limelight should be from the target.
         *                 If 0, the robot will likely be trying to enter a wall
         */
        public DistanceError( Limelight limelight, double distance ) {
            this.limelight = limelight;
            this.distance = distance;
        }

        @Override
        public void setPIDSourceType( PIDSourceType pidSource ) {
            if( pidSource != PIDSourceType.kDisplacement ) {
                throw new IllegalArgumentException("Limelight PID only accepts Displacement source type");
            }

        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }

        @Override
        public double pidGet() {
            return limelight.limelightDistance.get() - distance;
        }
    }

}
