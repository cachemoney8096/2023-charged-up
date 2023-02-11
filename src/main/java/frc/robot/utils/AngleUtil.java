package frc.robot.utils;

public class AngleUtil {
  /** Wraps angle to [0,360) */
  public static double wrapAngle(double angleDeg) {
    double wrappedAngleDeg = angleDeg % 360.0;
    if (wrappedAngleDeg < 0.0) {
      wrappedAngleDeg += 360.0;
    }
    return wrappedAngleDeg;
  }
}
