package frc.team1891.common.drivetrains.swervemodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class BSF_FalconSteerController implements SteerController {
    private final WPI_TalonFX steerMotor;
    private final CANCoder encoder;
    private final double steeringGearRatio;

    private final double encoderOffset;

    public BSF_FalconSteerController(WPI_TalonFX steerMotor, CANCoder encoder, double steeringGearRatio, double encoderOffset) {
        this.steerMotor = steerMotor;
        this.encoder = encoder;
        this.steeringGearRatio = steeringGearRatio;
        this.encoderOffset = encoderOffset;
    }

    private void configureSteerMotor() {
        steerMotor.configFactoryDefault();
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                true,
                25,
                40,
                .1);

        TalonFXConfiguration configuration = new TalonFXConfiguration();

//        configuration.slot0.kP = 1;
//        configuration.slot0.kI = 0;
//        configuration.slot0.kD = 0;
//        configuration.slot0.kF = 0;
        configuration.supplyCurrLimit = angleSupplyLimit;
        steerMotor.configAllSettings(configuration);
        steerMotor.setInverted(false);
        steerMotor.setNeutralMode(NeutralMode.Brake);
        resetToAbsolute();
    }

    private void resetToAbsolute(){
        double absolutePosition = SwerveModule.degreesToMotorEncoderTicks(getDegrees() - encoderOffset, 150/7., 2048);
        steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    @Override
    public void drive(SwerveModuleState state) {
        Rotation2d angle = state.angle;

//        steerMotor.set(ControlMode.Position, bsfSwerveModule.Conversions.degreesToFalcon(angle.getDegrees(), steeringGearRatio));
        steerMotor.set(ControlMode.Position, SwerveModule.degreesToMotorEncoderTicks(angle.getDegrees(), steeringGearRatio, 2048));
    }

    @Override
    public void stop() {
        steerMotor.stopMotor();
    }

    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d(SwerveModule.motorEncoderTicksToRadians(steerMotor.getSelectedSensorPosition(), steeringGearRatio, 2048));
    }

    @Override
    public double getRadians() {
        return SwerveModule.motorEncoderTicksToRadians(steerMotor.getSelectedSensorPosition(), steeringGearRatio, 2048);
    }

    @Override
    public double getDegrees() {
        return SwerveModule.motorEncoderTicksToDegrees(steerMotor.getSelectedSensorPosition(), steeringGearRatio, 2048);
    }
}
