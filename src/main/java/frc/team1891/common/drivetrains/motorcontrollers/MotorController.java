package frc.team1891.common.drivetrains.motorcontrollers;

public interface MotorController extends edu.wpi.first.wpilibj.motorcontrol.MotorController {
    /**
     * Resets the motor's position.
     */
    default void resetPosition() {
        setPosition(0);
    }

    /**
     * Sets the motor's position.
     * @param positionMeters meters
     */
    void setPosition(int positionMeters);

    /**
     * Returns the motor's position.
     * @return meters
     */
    double getPosition();

    /**
     * Drives the motor at the given velocity.
     * @param velocityMPS meters per second
     */
    void setVelocity(double velocityMPS);

    /**
     * Returns the motor's velocity.
     * @return meters per second
     */
    double getVelocity();
}
