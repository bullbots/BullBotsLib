package frc.team1891.common.hardware;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

@SuppressWarnings("unused")
public class SimNavX extends NavX {
    private final SimDouble simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));

    public SimNavX(NavXComType comType) {
        super(comType);
    }

    public SimNavX(NavXComType comType, NavXUpdateRate updateRate) {
        super(comType, updateRate);
    }

    public SimNavX(NavXComType comType, int customRateHz) {
        super(comType, customRateHz);
    }

    public void setDegrees(double degrees) {
        simAngle.set(degrees);
    }

    public void setRadians(double radians) {
        simAngle.set(Math.toDegrees(radians));
    }
}
