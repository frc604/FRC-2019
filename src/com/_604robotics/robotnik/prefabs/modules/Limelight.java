package com._604robotics.robotnik.prefabs.modules;

import static com._604robotics.robot2019.constants.Calibration.*;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Represents a Limelight vision camera.
 *
 * @see <a href="http://docs.limelightvision.io/en/latest/index.html">Official Documentation</a>
 */
public class Limelight extends Module {

  private NetworkTable table;
  private HttpCamera cameraStream;
  private int
      prevPipeline; // The previous vision pipeline in use, for when swapping back after driver mode

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

  /** Creates a new Limelight module with the default NetworkTable name of "limelight" */
  public Limelight() {
    this("limelight");
  }

  /**
   * Creates a Limelight module with a custom NetworkTable table. Allows for using multiple
   * Limelights on one robot. The name of the table must be changed in the settings of the Limelight
   * camera too.
   *
   * <p>WARNING: There is currently a global Calibration value for angle and height of the
   * limelight. If you are using multiple units, there must be distinct height and angle values for
   * each.
   *
   * @param tableName Name of the table to access the Limelight at
   */
  public Limelight(String tableName) {
    // TODO: Pass Height and angle values, instead of grabbing from Calibration to allow for
    // multiple limelights
    super(Limelight.class);
    this.table = NetworkTableInstance.getDefault().getTable(tableName);
    this.prevPipeline = Calibration.LIMELIGHT_VISION_PIPE;

    limelightHasTargets =
        addOutput(
            "limelightHasTargets",
            () -> table.getEntry("tv").getNumber((Number) 0).intValue() == 1);
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
    CameraServer.getInstance().addCamera(cameraStream);

    setDefaultAction(scan);
  }

  /**
   * Runs vision targeting on the limelight. This reduces exposure, so that the target is the only
   * thing that appears. Also changes to the last used pipeline that was used for scanning.
   */
  private class Scan extends Action {
    public Scan() {
      super(Limelight.this, Scan.class);
    }

    @Override
    public void begin() {
      table.getEntry("camMode").setNumber(0); // Sets threshold
      limelightPipeline.set(prevPipeline);
    }

    @Override
    public void run() {
      setSettings();
    }

    @Override
    public void end() {
      prevPipeline = limelightPipeline.get();
    }
  }

  /**
   * Runs driver vision mode on the limelight. This is a custom pipeline, with increased exposure so
   * that is is possible to actually see things on the stream. Also stops the limelight from trying
   * to detect objects.
   */
  private class Driver extends Action {

    public Driver() {
      super(Limelight.this, Driver.class);
    }

    @Override
    public void begin() {
      // When swapping to this action, we need to disable vision processing
      table.getEntry("camMode").setNumber(1);
      limelightPipeline.set(Calibration.LIMELIGHT_DRIVER_PIPE);
    }

    @Override
    public void run() {
      setSettings();
    }

    @Override
    public void end() {
      CameraServer.getInstance().removeCamera("limelight");
    }
  }

  public final Action scan = new Scan();
  public final Action driver = new Driver();

  /**
   * Checks the inputs to the limelight for a change, and if present, updates the limelight with the
   * new values.
   *
   * <p>Should be called in the run() of each action.
   */
  private void setSettings() {
    if (limelightPipeline.isFresh()) {
      table.getEntry("pipeline").setNumber(limelightPipeline.get());
    }

    if (limelightLED.isFresh()) {
      table.getEntry("ledMode").setNumber(limelightLED.get());
    }

    if (limelightSnapshotEnabled.isFresh()) {
      table.getEntry("snapshot").setNumber((limelightSnapshotEnabled.get() ? 1 : 0));
    }

    if (limelightStreamMode.isFresh()) {
      table.getEntry("stream").setNumber(limelightStreamMode.get());
    }
  }

  /**
   * Uses trig to find the distance of the limelight to the target. Requires that the limelight be
   * mounted at an angle relative to the target, the height of the target, and the height of the
   * limelight is known.
   *
   * @return Distance in the same units of the Calibration constants (should be inches)
   */
  public double getDistance() {
    return (Calibration.LIMELIGHT_HEIGHT - Calibration.TARGET_HEIGHT)
        * Math.tan(this.limelightY.get() + Calibration.LIMELIGHT_ANGLE);
  }
}
