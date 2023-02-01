package frc.team1891.common.drivetrains.swervemodules;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface DriveController {
    /**
     * Controls the drive motor to reach the velocity of the given state.
     * @param state the desired state
     */
    void drive(SwerveModuleState state);

    /**
     * Stops the drive motor.
     */
    void stop();

    /**
     * Position of the wheel in meters.
     * @return position of the wheel
     */
    double getPosition();

    /**
     * Velocity of the wheel in meters per second.
     * @return velocity of the wheel.
     */
    double getVelocity();
}
