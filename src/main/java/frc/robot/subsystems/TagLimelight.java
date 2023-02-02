// Code from team 3005

package frc.robot.subsystems;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Limelight to read april tags */
public class TagLimelight extends SubsystemBase {
  private final double kCameraAngleDegrees;
  private final double kCameraHeight;
  private final double kTargetHeight;
  private final double kImageCaptureLatency = 11.0;

  // Simulation functions
  private SimDevice m_simDevice;
  private SimDouble m_targetArea;
  private SimDouble m_skew;
  private SimDouble m_latency;
  private SimDouble m_tx;
  private SimDouble m_ty;
  private SimBoolean m_valid;

  // NT published variables when using translation api
  private double m_lastDistance = 0.0;
  private double m_lastX = 0.0;
  private double m_lastY = 0.0;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  /**
   * Create an IntakeLimelight object
   *
   * @param angleDegrees angle from normal in degress. Looking straight out is 0, and increasing as
   *     the camera is tilted towards the ceiling.
   * @param heightMeters height of the camera measured from the lens to the ground in meters.
   * @param targetHeightMeters height to the center of the target in meters
   */
  public TagLimelight(double angleDegrees, double heightMeters, double targetHeightMeters) {
    kCameraAngleDegrees = angleDegrees;
    kCameraHeight = heightMeters;
    kTargetHeight = targetHeightMeters;
    setLimelightValues(ledMode.ON, camMode.VISION_PROCESSING, pipeline.PIPELINE3);

    m_simDevice = SimDevice.create("Limelight");
    if (m_simDevice != null) {
      m_targetArea = m_simDevice.createDouble("Target Area", Direction.kBidir, 0.0);
      m_skew = m_simDevice.createDouble("Skew", Direction.kBidir, 0.0);
      m_latency = m_simDevice.createDouble("Latency", Direction.kBidir, 0.0);
      m_tx = m_simDevice.createDouble("Tx", Direction.kBidir, 0.0);
      m_ty = m_simDevice.createDouble("Ty", Direction.kBidir, 0.0);
      m_valid = m_simDevice.createBoolean("Valid", Direction.kBidir, false);
    }
  }

  /*
   * 0 - whatever pipleine is on
   * 1 - off
   * 2 - blinking
   * 3 - on
   */
  public static enum ledMode {
    PIPELINE_MODE,
    OFF,
    BLINK,
    ON
  }

  /*
   * 0 - limelight
   * 1 - driver
   */
  public static enum camMode {
    VISION_PROCESSING,
    DRIVER_CAMERA
  }

  public static enum pipeline {
    PIPELINE0,
    PIPELINE1,
    PIPELINE2,
    PIPELINE3,
    PIPELINE4,
    PIPELINE5,
    PIPELINE6,
    PIPELINE7,
    PIPELINE8,
    PIPELINE9
  }

  /**
   * Sets the led mode.
   *
   * @param mode LED operating mode.
   */
  public void setLedMode(ledMode mode) {
    table.getEntry("ledMode").setNumber(mode.ordinal());
  }

  /**
   * Sets the camera operating mode.
   *
   * @param mode Camera operating mode.
   */
  public void setCamMode(camMode mode) {
    table.getEntry("ledMode").setNumber(mode.ordinal());
  }

  /**
   * Sets the vision thresholding pipeline.
   *
   * @param line Pipeline index
   */
  public void setPipeline(pipeline line) {
    table.getEntry("ledMode").setNumber(line.ordinal());
  }

  public void setLimelightValues(ledMode ledMode, camMode camMode, pipeline line) {
    setLedMode(ledMode);
    setCamMode(camMode);
    setPipeline(line);

    SmartDashboard.putString("LED Mode", ledMode.name());
    SmartDashboard.putString("Cam Mode", camMode.name());
  }

  /**
   * @return validObject - Whether the limelight has any valid targets (0 or 1)
   */
  public double getValidTarget() {
    if (m_simDevice != null) {
      return m_valid.get() ? 1 : 0;
    }
    return table.getEntry("tv").getDouble(-1);
  }

  /**
   * @return xOffSet - Horizontal Offset(Left to Right) From Crosshair To Target (LL2: -29.8 to 29.8
   *     degrees)
   */
  public double getOffSetX() {
    if (m_simDevice != null) {
      return m_tx.get();
    }
    return table.getEntry("tx").getDouble(0.0);
  }

  /**
   * @return yOffSett - Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees
   *     | LL2: -24.85 to 24.85 degrees)
   */
  public double getOffSetY() {
    if (m_simDevice != null) {
      return m_ty.get();
    }
    return table.getEntry("ty").getDouble(0.0);
  }

  /**
   * @return targetArea - Target Area (0% of image to 100% of image)
   */
  public double getTargetArea() {
    if (m_simDevice != null) {
      return m_targetArea.get();
    }
    return table.getEntry("ta").getDouble(0.0);
  }

  /**
   * @return skew - Skew or rotation (-90 degrees to 0 degrees)
   */
  public double getSkew() {
    if (m_simDevice != null) {
      return m_skew.get();
    }
    return table.getEntry("ts").getDouble(0.0);
  }

  /**
   * @return latency - The pipeline’s latency contribution in seconds. Add at least 11ms for image
   *     capture latency.
   */
  public double getLatency() {
    if (m_simDevice != null) {
      return m_latency.get() + kImageCaptureLatency;
    }
    return (table.getEntry("tl").getDouble(0.0) + kImageCaptureLatency) / 1e3;
  }

  /**
   * Get the timestamp of the last update to the network table. This can be used to get a better
   * estimate of the total latency. That is (lastUpdate - latency).
   *
   * @return timestamp of the last update to the latency update in seconds.
   */
  public double getLastTimestamp() {
    if (m_simDevice != null) {
      return Timer.getFPGATimestamp();
    }
    return table.getEntry("tl").getLastChange() / 1e6;
  }

  /**
   * @return true is limelight has made a target else false
   */
  public boolean isValidTarget() {
    if (getValidTarget() > 0) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * @return true as long as limelight does not have value of -1
   */
  public boolean CheckConnection() { // ??? this depends on return of null, -1?
    if (getValidTarget() == -1.0) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * Calculate the lateral distance to the target in meters for a target other than the one defined
   * in the constructor.
   *
   * @param targetHeight height of the target in meters
   * @return distance to the target in meters
   */
  public double getDistanceFromTargetMeters(double targetHeight) {
    double angle = kCameraAngleDegrees + getOffSetY();
    if (angle < 1 || angle > 89) {
      return 0;
    }
    double tanTerm = Math.tan(Math.toRadians(angle));
    double distance = (targetHeight - kCameraHeight) / tanTerm;

    return distance;
  }

  /**
   * Get the lateral distance to the target as defined in the construtor.
   *
   * @return distance to the target in meters
   */
  public double getDistanceFromTargetMeters() {
    return getDistanceFromTargetMeters(kTargetHeight);
  }

  /**
   * Get a 2d translation from the camera to the target, including normalization to handle the
   * effects of angle to target. See the below discussion.
   * https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/7
   *
   * @parm targetHeightMeters use a target height other than what is provided in constructor.
   * @return Translation2d from the camera to the target. This is camera relative, x is out of the
   *     camera and positive y is to the left from the camera's point of view.
   */
  public Translation2d getTargetTranslation(double targetHeightMeters) {
    /**
     * This function uses the limelight coordinates until the end of the function. That is, the x
     * axis is left/right, horizontal to the field. The y axis is 'up/down' normal to the field
     * surface, and z is away from the camera, horizontal to the field surface.
     *
     * <p>The output coordinates are only a translation in x/y to the target, where x is out of the
     * camera, horizontal to the field, and y is positive to the left from the perspective of the
     * camera.
     */
    double diff_height = targetHeightMeters - kCameraHeight;

    // x is the "left/right" to the target from the camera
    double x = Math.tan(Math.toRadians(getOffSetX()));

    // y is the 'up/down' to the target from the center of the camera
    double y = Math.tan(Math.toRadians(getOffSetY()));

    // z is straight out of the camera
    double z = 1.0;
    double length = Math.sqrt(x * x + y * y + z * z);

    // Normalized vector components
    double x_norm = x / length;
    double y_norm = y / length;
    double z_norm = z / length;

    // Rotate vector by camera degrees around camera x axis
    Translation2d zy_translation =
        new Translation2d(z_norm, y_norm).rotateBy(Rotation2d.fromDegrees(kCameraAngleDegrees));
    z_norm = zy_translation.getX();
    y_norm = zy_translation.getY();

    /**
     * Find the intersection between the target in space, and the vector pointing to the target
     *
     * <p>This becomes a vector [x_norm, y_norm, z_norm] * d = [x_target, diff_height, z_target]
     *
     * <p>x_norm * d = x_target y_norm * d = diff_height z_target * d = z_target
     */
    double scaling = diff_height / y_norm;
    double x_target = scaling * x_norm;
    double z_target = scaling * z_norm;
    double distance = Math.hypot(z_target, x_target);

    // Convert camera coordinates to our conventions
    Translation2d result = new Translation2d(distance, new Rotation2d(z_target, -x_target));

    // For NT so this function doesn't need to be called multiple times
    m_lastDistance = distance;
    m_lastX = result.getX();
    m_lastY = result.getY();

    return result;
  }

  /**
   * Get a 2d translation from the camera to the target, including normalization to handle the
   * effects of angle to target. See the below discussion.
   * https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/7
   *
   * @return Translation2d from the camera to the target. This is camera relative, x is out of the
   *     camera and positive y is to the left from the camera's point of view.
   */
  public Translation2d getTargetTranslation() {
    return getTargetTranslation(kTargetHeight);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Target Area", () -> getTargetArea(), null);
    builder.addDoubleProperty("Skew", () -> getSkew(), null);
    builder.addDoubleProperty("Latency", () -> getLatency(), null);
    builder.addDoubleProperty("Distance", () -> getDistanceFromTargetMeters(), null);
    builder.addDoubleProperty("Tx", () -> getOffSetX(), null);
    builder.addDoubleProperty("Ty", () -> getOffSetY(), null);
    builder.addDoubleProperty("Translation Distance", () -> m_lastDistance, null);
    builder.addDoubleProperty("Translation X", () -> m_lastX, null);
    builder.addDoubleProperty("Translation Y", () -> m_lastY, null);
    builder.addBooleanProperty("Valid Target", () -> isValidTarget(), null);
  }
}
