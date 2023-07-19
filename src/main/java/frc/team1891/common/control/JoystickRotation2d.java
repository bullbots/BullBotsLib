// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.control;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.DoubleSupplier;

/** Takes the two axes of a joystick and converts it to an angle. */
@SuppressWarnings("unused")
public class JoystickRotation2d {
    DoubleSupplier xAxis, yAxis;
    public JoystickRotation2d(DoubleSupplier xAxis, DoubleSupplier yAxis) {
        this.xAxis = xAxis;
        this.yAxis = yAxis;
    }

    /**
     * Get the {@link Rotation2d} created by the pair of axis.
     */
    public Rotation2d get() {
        return new Rotation2d(xAxis.getAsDouble(), yAxis.getAsDouble());
    }
}
