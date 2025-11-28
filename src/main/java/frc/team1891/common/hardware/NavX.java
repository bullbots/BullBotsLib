// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.hardware;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * A wrapper for the NavX IMU providing additional convenience methods.
 */
@SuppressWarnings("unused")
public class NavX extends AHRS {
    /**
     * Constructs a NavX with the specified communication type.
     * @param comType the communication type
     */
    public NavX(NavXComType comType) {
        super(comType);
    }

    /**
     * Constructs a NavX with the specified communication type and update rate.
     * @param comType the communication type
     * @param updateRate the update rate
     */
    public NavX(NavXComType comType, NavXUpdateRate updateRate) {
        super(comType, updateRate);
    }

    /**
     * Constructs a NavX with the specified communication type and custom update rate.
     * @param comType the communication type
     * @param customRateHz the custom update rate in Hz
     */
    public NavX(NavXComType comType, int customRateHz) {
        super(comType, customRateHz);
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
