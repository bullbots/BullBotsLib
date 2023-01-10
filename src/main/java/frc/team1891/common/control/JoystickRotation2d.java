// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

/** Takes the two axes of a joystick and converts it to an angle. */
public class JoystickRotation2d {
    DoubleSupplier xAxis, yAxis;
    public JoystickRotation2d(DoubleSupplier xAxis, DoubleSupplier yAxis) {
        this.xAxis = xAxis;
        this.yAxis = yAxis;
    }

    public Rotation2d get() {
        return new Rotation2d(xAxis.getAsDouble(), yAxis.getAsDouble());
    }
}
