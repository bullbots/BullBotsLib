// Code based on 254's lib

package frc.team1891.common.hardware.lazy;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
@SuppressWarnings("unused")
public class LazyTalon extends Talon {
    protected double lastSet = Double.NaN;

    public LazyTalon(int deviceNumber) {
        super(deviceNumber);
    }

    public double getLastSet() {
        return lastSet;
    }

    @Override
    public void set(double value) {
        if (value != lastSet) {
            lastSet = value;
            super.set(value);
        } else {
            feed();
        }
    }
}