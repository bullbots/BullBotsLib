// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

/**
 * Structure to hold useful information about a {@link Drivetrain} and makes unit conversions easier.
 *
 * Used throughout BullBotsLib.
 */
public class DrivetrainConfig {
    public final double chassisMaxVelocityMetersPerSecond,
                        chassisMaxAccelerationMetersPerSecondSquared,
                        chassisMaxAngularVelocityRadiansPerSecond,
                        chassisMaxAngularAccelerationRadiansPerSecondSquared;
    public final double wheelRadiusMeters, gearRatio, encoderCountsPerMotorRevolution;

    /**
     * Creates a new drivetrain configuration
     * @param chassisMaxVelocityMetersPerSecond max attainable velocity of the drivetrain
     * @param chassisMaxAccelerationMetersPerSecondSquared max attainable acceleration of the drivetrain
     * @param chassisMaxAngularVelocityRadiansPerSecond max attainable angular velocity of the drivetrain
     * @param chassisMaxAngularAccelerationRadiansPerSecondSquared max attainable angular acceleration of the drivetrain
     * @param wheelRadiusMeters radius of the wheels
     * @param gearRatio gear ratio of wheels (motor rotations / wheel rotations)
     * @param encoderCountsPerMotorRevolution encoder counts of motor per revolution (e.g. TalonFX = 2048)
     */
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

    /**
     * Converts a position in meters into encoder ticks.
     * @param positionMeters position
     * @return position in encoder ticks
     */
    public int distanceToEncoderTicks(double positionMeters) {
        double wheelRotations = positionMeters / (wheelRadiusMeters * 2 * Math.PI);
        double motorRotations = wheelRotations * gearRatio;
        return (int)(motorRotations * encoderCountsPerMotorRevolution);
    }

    /**
     * Converts a position in encoder ticks into meters.
     * @param sensorCounts position
     * @return position in meters
     */
    public double encoderTicksToDistance(double sensorCounts) {
        double motorRotations = sensorCounts / encoderCountsPerMotorRevolution;
        double wheelRotations = motorRotations / gearRatio;
        return wheelRotations * (wheelRadiusMeters * 2 * Math.PI);
    }

    /**
     * Converts encoder ticks per 100ms into meters per second.
     * @param sensorCountsPer100ms velocity
     * @return velocity in meters per second
     */
    public double encoderTicksPer100msToVelocity(double sensorCountsPer100ms) {
        double motorRotationsPer100ms = sensorCountsPer100ms / encoderCountsPerMotorRevolution;
        double motorRotationsPerSecond = motorRotationsPer100ms * 10;
        double wheelRotationsPerSecond = motorRotationsPerSecond / gearRatio;
        return wheelRotationsPerSecond * (wheelRadiusMeters * 2 * Math.PI);
    }

    /**
     * Converts a velocity in m/s into encoder ticks per 100ms.
     * @param velocityMetersPerSecond velocity
     * @return velocity in encoder ticks per 100ms
     */
    public double velocityToEncoderTicksPer100ms(double velocityMetersPerSecond) {
        double wheelRotationsPerSecond = velocityMetersPerSecond / (wheelRadiusMeters * 2 * Math.PI);
        double motorRotationsPerSecond = wheelRotationsPerSecond * gearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / 10;
        double sensorCountsPer100ms = motorRotationsPer100ms * encoderCountsPerMotorRevolution;
        return sensorCountsPer100ms;
    }
}
