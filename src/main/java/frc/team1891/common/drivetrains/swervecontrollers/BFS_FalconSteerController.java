package frc.team1891.common.drivetrains.swervecontrollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Based on https://github.com/Team364/BaseFalconSwerve
 */
@SuppressWarnings("unused")
public class BFS_FalconSteerController implements SteerController {
    private final WPI_TalonFX steerMotor;
    private final WPI_CANCoder encoder;
    private final double steeringGearRatio;

    private final double encoderOffsetDegrees;

    // Uses the gear ratio from an MK4i module
    public BFS_FalconSteerController(WPI_TalonFX steerMotor, WPI_CANCoder encoder, double encoderOffsetDegrees) {
        this(steerMotor, encoder, (150/7.), encoderOffsetDegrees);
    }

    /**
     * Creates a new SteerController for a Falcon500
     * @param steerMotor the Falcon500 steer motor for the module
     * @param encoder the CANCoder for the module
     * @param steeringGearRatio the gear ratio between the steer motor and the wheel steering
     * @param encoderOffsetDegrees the offset of the CANCoder measurement.
     *                             The measurement of the CANCoder when the module is facing forward
     */
    public BFS_FalconSteerController(WPI_TalonFX steerMotor, WPI_CANCoder encoder, double steeringGearRatio, double encoderOffsetDegrees) {
        this.steerMotor = steerMotor;
        this.encoder = encoder;
        this.steeringGearRatio = steeringGearRatio;
        this.encoderOffsetDegrees = encoderOffsetDegrees;

        calibrateEncoders();
    }

    /**
     * Aligns the TalonFX encoder measurement with the measurement from the CANCoder.
     *
     * This is called when the robot turns on.
     */
    public void calibrateEncoders(){
        double absolutePosition = SwerveModule.degreesToMotorEncoderTicks(encoder.getAbsolutePosition() - encoderOffsetDegrees, steeringGearRatio, 2048);
        steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    @Override
    public void drive(SwerveModuleState state) {
        Rotation2d angle = state.angle;
        double angleDegrees = placeInAppropriate0To360Scope(getDegrees(), angle.getDegrees());

        steerMotor.set(ControlMode.Position, SwerveModule.degreesToMotorEncoderTicks(angleDegrees, steeringGearRatio, 2048));
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

    /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }
}
