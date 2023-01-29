package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import java.util.function.BooleanSupplier;

/** Mostly borrowed from 3005 */
public class SparkMaxUtils {

  /**
   * @param error API return value
   * @return
   */
  public static int check(REVLibError error) {
    return error == REVLibError.kOk ? 0 : 1;
  }

  public static class UnitConversions {

    public static REVLibError setDegreesFromGearRatio(
        AbsoluteEncoder sparkMaxEncoder, double ratio) {
      double degreesPerRotation = 360.0 / ratio;
      double degreesPerRotationPerSecond = degreesPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(degreesPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(degreesPerRotationPerSecond);
    }

    public static REVLibError setRadsFromGearRatio(AbsoluteEncoder sparkMaxEncoder, double ratio) {
      double radsPerRotation = (2.0 * Math.PI) / ratio;
      double radsPerRotationPerSecond = radsPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(radsPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(radsPerRotationPerSecond);
    }

    public static REVLibError setDegreesFromGearRatio(
        RelativeEncoder sparkMaxEncoder, double ratio) {
      double degreesPerRotation = 360.0 / ratio;
      double degreesPerRotationPerSecond = degreesPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(degreesPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(degreesPerRotationPerSecond);
    }

    public static REVLibError setRadsFromGearRatio(RelativeEncoder sparkMaxEncoder, double ratio) {
      double radsPerRotation = (2.0 * Math.PI) / ratio;
      double radsPerRotationPerSecond = radsPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(radsPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(radsPerRotationPerSecond);
    }
  }

  public static String faultWordToString(short faults) {
    if (faults == 0) {
      return "";
    }

    StringBuilder builder = new StringBuilder();
    int faultsInt = faults;
    for (int i = 0; i < 16; i++) {
      if (((1 << i) & faultsInt) != 0) {
        builder.append(CANSparkMax.FaultID.fromId(i).toString());
        builder.append(" ");
      }
    }
    return builder.toString();
  }

  /**
   * Takes a function returning true on success, and tries running it until it succeeds up to the
   * max retry attempts
   */
  public static void initWithRetry(BooleanSupplier initFunction, int maxRetryAttempts) {
    int numAttempts = 0;
    while (!initFunction.getAsBoolean()) {
      numAttempts++;
      if (numAttempts > maxRetryAttempts) {
        break;
      }
    }
  }
}
