package frc.team1891.common;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {
    protected abstract void configureSmartDashboard();

    /**
     * Stops all moving parts.
     */
    public abstract void stop();

    /**
     * Call getFirmwareVersion() on all CAN Devices and verify that it gives a valid result.
     * @return if all devices are properly connected over CAN
     */
    public abstract boolean checkAllCANDevices();
}
