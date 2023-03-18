// Code based on 254's lib

package frc.team1891.common.hardware.lazy;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

@SuppressWarnings("unused")
public class LazySolenoid extends Solenoid {
    private boolean lastValue = false;

    public LazySolenoid(PneumaticsModuleType moduleType, int channel) {
        super(moduleType, channel);
        set(false);
    }

    public LazySolenoid(int module, PneumaticsModuleType moduleType, int channel) {
        super(module, moduleType, channel);
        set(false);
    }


    @Override
    public void set(boolean on) {
        if (on != lastValue) {
            super.set(on);
            lastValue = on;
        }
    }
}
