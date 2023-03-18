package frc.team1891.common.hardware;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;

@SuppressWarnings("unused")
public class SimNavX extends NavX {
    private final SimDouble simAngle;
    public SimNavX() {
        super();

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    }

    public void setDegrees(double degrees) {
        simAngle.set(degrees);
    }

    public void setRadians(double radians) {
        simAngle.set(Math.toDegrees(radians));
    }
}

