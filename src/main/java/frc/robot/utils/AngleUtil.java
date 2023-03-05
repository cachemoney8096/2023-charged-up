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

  
  /** Wraps angle to [-180,180) */
  public static double wrapAngleAroundZero(double angleDeg) {
    double wrappedAngleDeg = angleDeg % 360.0;
    if (wrappedAngleDeg < 0.0) {
      wrappedAngleDeg += 360.0;
    }
    if (wrappedAngleDeg > 180) {
      wrappedAngleDeg -= 360.0;
    }
    return wrappedAngleDeg;
  }
}
