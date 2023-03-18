package frc.robot.utils;

import edu.wpi.first.math.filter.MedianFilter;

/** Checks Absolute Encoders for Noise */
public class AbsoluteEncoderChecker {
  private double[] lastFive;
  private int index;
  private MedianFilter medianFilter;
  private double lastCalculatedMedian;

  public AbsoluteEncoderChecker() {
    this.lastFive = new double[5];
    this.index = 0;
    this.medianFilter = new MedianFilter(5);
  }

  public void addReading(double lastReading) {
    lastFive[index] = lastReading;
    index++;
    if (index == 5) {
      index = 0;
    }
    lastCalculatedMedian = medianFilter.calculate(lastReading);
  }

  /**
   * @return true if any of the five readings are different, otherwise returns false
   */
  public boolean encoderConnected() {
    for (int i = 1; i < lastFive.length; i++) {
      if (lastFive[i] != lastFive[0]) {
        return true;
      }
    }
    return false;
  }

  /**
   * @return the median of the five current values
   */
  public double getMedian() {
    return lastCalculatedMedian;
  }
}
