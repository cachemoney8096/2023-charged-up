package frc.robot.utils;

import edu.wpi.first.math.filter.MedianFilter;

/** Checks Absolute Encoders for Noise */
public class AbsoluteEncoderChecker {
    private double[] lastFive;
    private int index;
    private MedianFilter medianFilter;

    public AbsoluteEncoderChecker() {
        this.lastFive = new double[5];
        this.index = 0;
        this.medianFilter = new MedianFilter(5);
    }

    public void addReading(double lastReading) {
        lastFive[index] = lastReading;
        index ++;
        if (index == 5) {
            index = 0;
        }
    }

    /** @return true if all of the last five absolute encoder readings were equal, false if otherwise*/
    public boolean checkLastFive() {
        for (int i = 1; i < lastFive.length; i ++) {
            if (lastFive[i] != lastFive[0]) {
                return false;
            }
        }
        return true;
    }

    /** @return the median of the five current values */
    public double getMedian() {
        // resets so only the most up-to-date values are checked
        medianFilter.reset();

        // adds each of the first four readings to the medianFilter window
        for (int i = 0; i < lastFive.length-1; i ++) {
            medianFilter.calculate(lastFive[i]);
        }

        // returns the median of the window after adding the last reading
        return medianFilter.calculate(lastFive[lastFive.length-1]);
    }
}
