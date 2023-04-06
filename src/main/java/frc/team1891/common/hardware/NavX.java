// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

@SuppressWarnings("unused")
public class NavX extends AHRS {
    public NavX(SPI.Port spi_port_id, byte update_rate_hz) {
        super(spi_port_id, update_rate_hz);
    }

    public NavX(SPI.Port spi_port_id, int spi_bitrate, byte update_rate_hz) {
        super(spi_port_id, spi_bitrate, update_rate_hz);
    }

    public NavX(I2C.Port i2c_port_id, byte update_rate_hz) {
        super(i2c_port_id, update_rate_hz);
    }

    public NavX(SerialPort.Port serial_port_id, SerialDataType data_type, byte update_rate_hz) {
        super(serial_port_id, data_type, update_rate_hz);
    }

    public NavX(){
        super();
    }

    public NavX(SPI.Port spi_port_id){
        super(spi_port_id);
    }

    public NavX(I2C.Port i2c_port_id){
        super(i2c_port_id);
    }

    public NavX(SerialPort.Port serial_port_id) {
        super(serial_port_id);
    }

    /**
     * Returns the angle in degrees.
     * @return angle
     */
    public double getDegrees() {
        return getAngle();
    }

    /**
     * Returns the angle in degrees bound between 0 and 360.
     * @return angle
     */
    public double getDegreesLooped() {
        double angle = getDegrees() % 360.;
        angle = angle < 0 ? angle + 360 : angle;
        return angle;
    }

    /**
     * Returns the angle in radians.
     * @return angle
     */
    public double getRadians() {
        return Math.toRadians(getAngle());
    }

    /**
     * Returns the angle in radians bound between 0 and 2 PI.
     * @return angle
     */
    public double getRadiansLooped() {
        double angle = getRadians() % (2*Math.PI);
        angle = angle < 0 ? angle + (2*Math.PI) : angle;
        return angle;
    }
}
