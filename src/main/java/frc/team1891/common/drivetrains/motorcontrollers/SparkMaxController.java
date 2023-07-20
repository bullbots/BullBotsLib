package frc.team1891.common.drivetrains.motorcontrollers;

import com.revrobotics.RelativeEncoder;
import frc.team1891.common.drivetrains.DrivetrainConfig;
import frc.team1891.common.hardware.WPI_CANSparkMax;

/**
 * An implementation of {@link MotorController} assuming native encoder units (rotations and rotations / second).
 */
@SuppressWarnings("unused")
public class SparkMaxController implements MotorController {
    private final WPI_CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final DrivetrainConfig config;

    public SparkMaxController(WPI_CANSparkMax motor, DrivetrainConfig config) {
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.config = config;
    }

    @Override
    public void setPosition(int positionMeters) {
        encoder.setPosition(config.distanceToRotations(positionMeters));
    }

    @Override
    public double getPosition() {
        return config.rotationsToDistance(encoder.getPosition());
    }

    @Override
    public void setVelocity(double velocityMPS) {
        this.set(velocityMPS / config.chassisMaxVelocityMetersPerSecond());
    }

    @Override
    public double getVelocity() {
        return config.rotationsPerMinuteToVelocity(encoder.getVelocity());
    }

    @Override
    public void set(double speed) {
        motor.set(speed);
    }

    @Override
    public double get() {
        return motor.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return motor.getInverted();
    }

    @Override
    public void disable() {
        motor.disable();
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }
}
