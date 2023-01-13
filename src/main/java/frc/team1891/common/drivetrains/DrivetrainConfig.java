// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import edu.wpi.first.math.util.Units;

/** Drivetrain constraints. */
public class DrivetrainConfig {
    public final double chassisMaxVelocityMetersPerSecond,
                        chassisMaxAccelerationMetersPerSecondSquared,
                        chassisMaxAngularVelocityRadiansPerSecond,
                        chassisMaxAngularAccelerationRadiansPerSecondSquared;
    public final double wheelRadiusMeters, gearRatio, encoderCountsPerMotorRevolution;

    private final int k100msPerSecond = 10;

    public DrivetrainConfig(
        double chassisMaxVelocityMetersPerSecond,
        double chassisMaxAccelerationMetersPerSecondSquared,
        double chassisMaxAngularVelocityRadiansPerSecond,
        double chassisMaxAngularAccelerationRadiansPerSecondSquared,
        double wheelRadiusMeters,
        double gearRatio,
        double encoderCountsPerMotorRevolution
    ) {
        this.chassisMaxVelocityMetersPerSecond = chassisMaxVelocityMetersPerSecond;
        this.chassisMaxAccelerationMetersPerSecondSquared = chassisMaxAccelerationMetersPerSecondSquared;
        this.chassisMaxAngularVelocityRadiansPerSecond = chassisMaxAngularVelocityRadiansPerSecond;
        this.chassisMaxAngularAccelerationRadiansPerSecondSquared = chassisMaxAngularAccelerationRadiansPerSecondSquared;
        this.wheelRadiusMeters = wheelRadiusMeters;
        this.gearRatio = gearRatio;
        this.encoderCountsPerMotorRevolution = encoderCountsPerMotorRevolution;
    }

    public int distanceToNativeUnits(double positionMeters) {
        double wheelRotations = positionMeters / (wheelRadiusMeters * 2 * Math.PI);
        double motorRotations = wheelRotations * gearRatio;
        return (int)(motorRotations * encoderCountsPerMotorRevolution);
    }

    public double nativeUnitsToVelocityMeters(double sensorCountsPer100ms) {
        double motorRotationsPer100ms = sensorCountsPer100ms / encoderCountsPerMotorRevolution;
        double motorRotationsPerSecond = motorRotationsPer100ms * k100msPerSecond;
        double wheelRotationsPerSecond = motorRotationsPerSecond / gearRatio;
        return wheelRotationsPerSecond * (wheelRadiusMeters * 2 * Math.PI);
    }

    public double nativeUnitsToDistanceMeters(double sensorCounts) {
        double motorRotations = sensorCounts / encoderCountsPerMotorRevolution;
        double wheelRotations = motorRotations / gearRatio;
        return wheelRotations * (wheelRadiusMeters * 2 * Math.PI);
    }

    public double nativeUnitsToDistanceFeet(double sensorCounts) {
        double motorRotations = sensorCounts / encoderCountsPerMotorRevolution;
        double wheelRotations = motorRotations / gearRatio;
        return wheelRotations * Units.metersToFeet(wheelRadiusMeters) * 2 * Math.PI;
    }
}
