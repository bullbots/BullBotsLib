package frc.team1891.common.hardware;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * A simulated NavX for use in robot simulation.
 */
@SuppressWarnings("unused")
public class SimNavX extends NavX {
    private final SimDouble simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));

    /**
     * Constructs a SimNavX with the specified communication type.
     * @param comType the communication type
     */
    public SimNavX(NavXComType comType) {
        super(comType);
    }

    /**
     * Constructs a SimNavX with the specified communication type and update rate.
     * @param comType the communication type
     * @param updateRate the update rate
     */
    public SimNavX(NavXComType comType, NavXUpdateRate updateRate) {
        super(comType, updateRate);
    }

    /**
     * Constructs a SimNavX with the specified communication type and custom update rate.
     * @param comType the communication type
     * @param customRateHz the custom update rate in Hz
     */
    public SimNavX(NavXComType comType, int customRateHz) {
        super(comType, customRateHz);
    }

    /**
     * Sets the simulated angle in degrees.
     * @param degrees the angle in degrees
     */
    public void setDegrees(double degrees) {
        simAngle.set(degrees);
    }

    /**
     * Sets the simulated angle in radians.
     * @param radians the angle in radians
     */
    public void setRadians(double radians) {
        simAngle.set(Math.toDegrees(radians));
    }
}
