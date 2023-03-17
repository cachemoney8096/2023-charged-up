package frc.robot.utils;

/** Checks Absolute Encoders for Noise */
public class AbsoluteEncoderChecker {
    private double[] lastFive;
    private int index;

    public AbsoluteEncoderChecker() {
        this.lastFive = new double[5];
        this.index = 0;
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
}
