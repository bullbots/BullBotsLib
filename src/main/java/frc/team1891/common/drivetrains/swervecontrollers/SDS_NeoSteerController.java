package frc.team1891.common.drivetrains.swervecontrollers;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team1891.common.hardware.WPI_CANSparkMax;

/**
 * An implementation of the {@link SteerController} interface, intended to drive a Neo and use a CANCoder as the
 * absolute encoder.
 *
 * A direct copy of how SDS would control a steer Neo.
 */
@SuppressWarnings("unused")
public class SDS_NeoSteerController implements SteerController {
    private final WPI_CANSparkMax steerMotor;
    private final CANCoder encoder;

    private final RelativeEncoder motorEncoder;
    private final SparkMaxPIDController motorPIDController;

    private double resetIteration = 0;

    public SDS_NeoSteerController(WPI_CANSparkMax steerMotor, CANCoder encoder, double steeringGearRatio) {
        this.steerMotor = steerMotor;
        this.encoder = encoder;

        this.steerMotor.setIdleMode(WPI_CANSparkMax.IdleMode.kBrake);
        this.steerMotor.enableVoltageCompensation(12);
        this.steerMotor.setSmartCurrentLimit(20);
        this.motorEncoder = steerMotor.getEncoder();
        motorEncoder.setPositionConversionFactor(2.0 * Math.PI * steeringGearRatio);
        motorEncoder.setVelocityConversionFactor(2.0 * Math.PI * steeringGearRatio / 60.0);
        motorEncoder.setPosition(getRadians());
        motorPIDController = steerMotor.getPIDController();
        motorPIDController.setP(1.0);
        motorPIDController.setI(0.0);
        motorPIDController.setD(0.1);
        motorPIDController.setFeedbackDevice(motorEncoder);
    }

    @Override
    public void drive(SwerveModuleState state) {
        double referenceAngleRadians = state.angle.getRadians();
        // From SDS:
        double currentAngleRadians = motorEncoder.getPosition();

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (motorEncoder.getVelocity() < Math.toRadians(.5)) {
            if (++resetIteration >= 500) {
                resetIteration = 0;
                double absoluteAngle = getRadians();
                motorEncoder.setPosition(absoluteAngle);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            resetIteration = 0;
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        // this.referenceAngleRadians = referenceAngleRadians;

        motorPIDController.setReference(adjustedReferenceAngleRadians, WPI_CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void stop() {
        steerMotor.stopMotor();
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }

    @Override
    public double getRadians() {
        return Math.toRadians(encoder.getAbsolutePosition());
    }

    @Override
    public double getDegrees() {
        return encoder.getAbsolutePosition();
    }
}
