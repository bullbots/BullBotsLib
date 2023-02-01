package frc.team1891.common.drivetrains.swervemodules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SteerController {
    /**
     * Controls the steer motor to reach the angle of the given state.
     * @param state the desired state
     */
    void drive(SwerveModuleState state);

    /**
     * Stops the steer motor.
     */
    void stop();

    /**
     * Returns the angle of the module as a {@link Rotation2d}.
     * @return angle of the module
     */
    Rotation2d getRotation2d();

    /**
     * Returns the angle of the module in radians.
     * @return angle of the module
     */
    double getRadians();

    /**
     * Returns the angle of the module in degrees.
     * @return angle of the module
     */
    double getDegrees();
}
