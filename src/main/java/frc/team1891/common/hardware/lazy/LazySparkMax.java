package frc.team1891.common.hardware.lazy;

import frc.team1891.common.hardware.WPI_CANSparkMax;

public class LazySparkMax extends WPI_CANSparkMax {
    protected double lastSet = Double.NaN;

    public LazySparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    public double getLastSet() {
        return lastSet;
    }

    @Override
    public void set(double speed) {
        if (speed != lastSet) {
            super.set(speed);
        }
    }
}
