// Code based on 254's lib

package frc.team1891.common.hardware.lazy;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

@SuppressWarnings("unused")
public class LazyDoubleSolenoid extends DoubleSolenoid {
    private Value lastValue = Value.kOff;

    public LazyDoubleSolenoid(PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel) {
        super(moduleType, forwardChannel, reverseChannel);
    }

    public LazyDoubleSolenoid(int module, PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel) {
        super(module, moduleType, forwardChannel, reverseChannel);
    }

    @Override
    public void set(Value value) {
        if (value != lastValue) {
            super.set(value);
            lastValue = value;
        }
    }

    @Override
    public void toggle() {
        super.toggle();
        lastValue = get();
    }
}
