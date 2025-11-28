// Code based on 254's lib

package frc.team1891.common.hardware.lazy;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * A Solenoid that only updates when the value changes to reduce CAN bus traffic.
 */
@SuppressWarnings("unused")
public class LazySolenoid extends Solenoid {
    private boolean lastValue = false;

    /**
     * Constructs a LazySolenoid.
     * @param moduleType the pneumatics module type
     * @param channel the channel number
     */
    public LazySolenoid(PneumaticsModuleType moduleType, int channel) {
        super(moduleType, channel);
        super.set(false);
    }

    /**
     * Constructs a LazySolenoid.
     * @param module the module number
     * @param moduleType the pneumatics module type
     * @param channel the channel number
     */
    public LazySolenoid(int module, PneumaticsModuleType moduleType, int channel) {
        super(module, moduleType, channel);
        super.set(false);
    }


    @Override
    public void set(boolean on) {
        if (on != lastValue) {
            super.set(on);
            lastValue = on;
        }
    }
}
