package frc.team1891.common.drivetrains.swervemodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team1891.common.drivetrains.DrivetrainConfig;

/**
 * An implementation of the {@link DriveController} interface, intended to drive a falcon motor.
 */
public class FalconDriveController implements DriveController {
    private final WPI_TalonFX driveMotor;
    private final DrivetrainConfig config;

    private final SimpleMotorFeedforward feedforward;

    public FalconDriveController(WPI_TalonFX driveMotor, DrivetrainConfig config) {
        this(driveMotor, config, 1/config.chassisMaxVelocityMetersPerSecond);
    }

    public FalconDriveController(WPI_TalonFX driveMotor, DrivetrainConfig config, double staticFeedForward) {
        this.driveMotor = driveMotor;
        this.config = config;

         feedforward = new SimpleMotorFeedforward(staticFeedForward, 0, 0);
    }

    @Override
    public void drive(SwerveModuleState state) {
        double motorVelocity = config.velocityToEncoderTicksPer100ms(state.speedMetersPerSecond);
        driveMotor.set(ControlMode.Velocity, motorVelocity, DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond));
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
    }

    @Override
    public double getPosition() {
        return config.encoderTicksToDistance(driveMotor.getSelectedSensorPosition());
    }

    @Override
    public double getVelocity() {
        return config.encoderTicksPer100msToVelocity(driveMotor.getSelectedSensorVelocity());
    }
}
