// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.hardware;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Wrapper on AnalogInput for Pressure Sensor */
@SuppressWarnings("unused")
public class AnalogPressureSensor extends AnalogInput {
    private final double minVoltage, maxVoltage, minPressure, maxPressure;

    /**
     * Creates a new AnalogPressureSensor
     * @param channel analog channel on RoboRIO
     * @param minVoltage min voltage received
     * @param maxVoltage max voltage received
     * @param minPressure min pressure rating of sensor
     * @param maxPressure max pressure rating of sensor
     */
    public AnalogPressureSensor(int channel, double minVoltage, double maxVoltage, double minPressure, double maxPressure) {
        super(channel);

        this.minVoltage = minVoltage;
        this.maxVoltage = maxVoltage;
        this.minPressure = minPressure;
        this.maxPressure = maxPressure;

        if (RobotBase.isSimulation()) {
            SmartDashboard.putNumber("AnalogPressureSensor/Sim Pressure " + channel, 0);
        }
    }

    /**
     * Get the pressure of the analog sensor based on the voltage.
     * @return pressure in PSI
     */
    public double getPressure() {
        if (RobotBase.isReal()) {
            return Math.round(map(super.getVoltage(), minVoltage, maxVoltage, minPressure, maxPressure));
        } else {
            return SmartDashboard.getNumber("AnalogPressureSensor/Sim Pressure " + getChannel(), 0);
        }
    }

    /**
     * Get the pressure of the analog sensor based on the average voltage.
     * @return pressure in PSI
     */
    public double getAveragePressure() {
        return map(super.getAverageVoltage(), minVoltage, maxVoltage, minPressure, maxPressure);
    }

    private static double map(double num, double min1, double max1, double min2, double max2) {
        num -= min1;
        num /= (max1 - min1);
        num *= (max2 - min2);
        num += min2;
        return num;
    }
}
