// Code based on 254's lib

package frc.team1891.common.hardware.lazy;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
@SuppressWarnings("unused")
public class LazyTalonFX extends WPI_TalonFX {
    protected double lastSet = Double.NaN;
    protected TalonFXControlMode lastControlMode = null;

    public LazyTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    public double getLastSet() {
        return lastSet;
    }

    @Override
    public void set(TalonFXControlMode mode, double value) {
        if (value != lastSet || mode != lastControlMode) {
            lastSet = value;
            lastControlMode = mode;
            super.set(mode, value);
        }
        // Motor safety is fed twice if the value is changed.  Oh, well?
        feed();
    }
}