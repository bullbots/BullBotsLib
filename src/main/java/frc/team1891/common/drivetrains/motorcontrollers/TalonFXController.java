package frc.team1891.common.drivetrains.motorcontrollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.team1891.common.drivetrains.DrivetrainConfig;

/**
 * An implementation of {@link MotorController} for the {@link WPI_TalonFX}.
 */
@SuppressWarnings("unused")
public class TalonFXController implements MotorController {
    private final WPI_TalonFX motor;
    private final DrivetrainConfig config;
    
    public TalonFXController(WPI_TalonFX motor, DrivetrainConfig config) {
        this.motor = motor;
        this.config = config;
    }
    
    @Override
    public void setPosition(int positionMeters) {
        motor.setSelectedSensorPosition(config.distanceToEncoderTicks(positionMeters));
    }

    @Override
    public double getPosition() {
        return config.encoderTicksToDistance(motor.getSelectedSensorPosition());
    }

    @Override
    public double getVelocity() {
        return config.encoderTicksPer100msToVelocity(motor.getSelectedSensorVelocity());
    }

    @Override
    public void setVelocity(double velocityMPS) {
        motor.set(ControlMode.Velocity, config.velocityToEncoderTicksPer100ms(velocityMPS));
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
