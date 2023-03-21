package frc.robot.utils;

import edu.wpi.first.math.filter.MedianFilter;

/** Checks Absolute Encoders for Noise */
public class AbsoluteEncoderChecker {
  private double[] buffer;
  private int index;
  private MedianFilter medianFilter;
  private double lastCalculatedMedian = 0.0;
  private static final int NUM_READINGS = 20;

  public AbsoluteEncoderChecker() {
    this.buffer = new double[NUM_READINGS];
    this.index = 0;
    this.medianFilter = new MedianFilter(NUM_READINGS);
  }

  public void addReading(double lastReading) {
    buffer[index] = lastReading;
    index++;
    if (index == NUM_READINGS) {
      index = 0;
    }
    lastCalculatedMedian = medianFilter.calculate(lastReading);
  }

  /**
   * @return true if any of the buffeed readings are different, otherwise returns false
   */
  public boolean encoderConnected() {
    for (int i = 1; i < buffer.length; i++) {
      if (buffer[i] != buffer[0]) {
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
