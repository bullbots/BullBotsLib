package frc.team1891.common.hardware.lazy;

import frc.team1891.common.hardware.WPI_CANSparkMax;

/**
 * A SparkMax motor controller that only updates when the value changes to reduce CAN bus traffic.
 */
@SuppressWarnings("unused")
public class LazySparkMax extends WPI_CANSparkMax {
    /** The last speed value set to the motor controller. */
    protected double lastSet = Double.NaN;

    /**
     * Constructs a LazySparkMax.
     * @param deviceId the CAN device ID
     * @param type the motor type (brushed or brushless)
     */
    public LazySparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    /**
     * Gets the last speed value set to the motor controller.
     * @return the last speed value
     */
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
