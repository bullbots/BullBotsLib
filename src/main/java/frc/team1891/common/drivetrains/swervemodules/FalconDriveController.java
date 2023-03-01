package frc.team1891.common.drivetrains.swervemodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team1891.common.drivetrains.DrivetrainConfig;

/**
 * An implementation of the {@link DriveController} interface, intended to drive a falcon motor.
 */
public class FalconDriveController implements DriveController {
    protected final WPI_TalonFX driveMotor;
    protected final DrivetrainConfig config;

    /**
     * Creates a new FalconDriveController with the given CAN ID and Drivetrain config and createa a new WPI_TalonFX and
     * sets the PIDF to the given parameters.
     * @param driveMotorCANID the CAN ID of drive Falcon500
     * @param config drivetrain config
     */
    public FalconDriveController(int driveMotorCANID, DrivetrainConfig config, double kP, double kI, double kD, double kFF) {
        this(new WPI_TalonFX(driveMotorCANID), config);
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.config_kP(0, kP);
        driveMotor.config_kI(0, kI);
        driveMotor.config_kD(0, kD);
        driveMotor.config_kF(0, kFF);
    }

    /**
     * Creates a new FalconDriveController with the given motor and Drivetrain config, assuming that the PIDF of the
     * motor will be tuned elsewhere.
     * @param driveMotor drive Falcon500
     * @param config drivetrain config
     */
    public FalconDriveController(WPI_TalonFX driveMotor, DrivetrainConfig config) {
        this.driveMotor = driveMotor;
        this.config = config;
    }

    @Override
    public void drive(SwerveModuleState state) {
        double motorVelocity = config.velocityToEncoderTicksPer100ms(state.speedMetersPerSecond);
        driveMotor.set(ControlMode.Velocity, motorVelocity);
        // https://v5.docs.ctr-electronics.com/en/stable/ch16_ClosedLoop.html#motion-magic-position-velocity-current-closed-loop-closed-loop
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
