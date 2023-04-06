package frc.team1891.common.hardware;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

@SuppressWarnings("unused")
public class SimNavX extends NavX {
    private final SimDouble simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));

    public SimNavX(SPI.Port spi_port_id, byte update_rate_hz) {
        super(spi_port_id, update_rate_hz);
    }

    public SimNavX(SPI.Port spi_port_id, int spi_bitrate, byte update_rate_hz) {
        super(spi_port_id, spi_bitrate, update_rate_hz);
    }

    public SimNavX(I2C.Port i2c_port_id, byte update_rate_hz) {
        super(i2c_port_id, update_rate_hz);
    }

    public SimNavX(SerialPort.Port serial_port_id, SerialDataType data_type, byte update_rate_hz) {
        super(serial_port_id, data_type, update_rate_hz);
    }

    public SimNavX(){
        super();
    }

    public SimNavX(SPI.Port spi_port_id){
        super(spi_port_id);
    }

    public SimNavX(I2C.Port i2c_port_id){
        super(i2c_port_id);
    }

    public SimNavX(SerialPort.Port serial_port_id) {
        super(serial_port_id);
    }

    public void setDegrees(double degrees) {
        simAngle.set(degrees);
    }

    public void setRadians(double radians) {
        simAngle.set(Math.toDegrees(radians));
    }
}
