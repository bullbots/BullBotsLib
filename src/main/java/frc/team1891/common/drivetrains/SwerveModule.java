// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import frc.team1891.common.drivetrains.DrivetrainConfig;

public class SwerveModule {
    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonFX m_turningMotor;

    private final Encoder m_driveEncoder;
    private final Encoder m_turningEncoder;

    private final PIDController m_drivePIDController =
            new PIDController(1, 0, 0); // TODO: tune PID

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController m_turningPIDController;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel      The channel of the drive motor.
     * @param turningMotorChannel    The channel of the turning motor.
     * @param driveEncoderChannels   The channels of the drive encoder.
     * @param turningEncoderChannels The channels of the turning encoder.
     * @param driveEncoderReversed   Whether the drive encoder is reversed.
     * @param turningEncoderReversed Whether the turning encoder is reversed.
     */
    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            int[] driveEncoderChannels,
            int[] turningEncoderChannels,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed,
            DrivetrainConfig config) {
        m_turningPIDController =
                new ProfiledPIDController(
                        1, 0, 0, // TODO: tune PID
                        new TrapezoidProfile.Constraints(
                                config.chassisMaxAngularVelocityRadiansPerSecond,
                                config.chassisMaxAngularAccelerationRadiansPerSecondSquared
                        ));

        m_driveMotor = new WPI_TalonFX(driveMotorChannel);
        m_turningMotor = new WPI_TalonFX(turningMotorChannel);

        m_driveEncoder = new Encoder(driveEncoderChannels[0], driveEncoderChannels[1]);

        m_turningEncoder = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveEncoder.setDistancePerPulse((2 * config.wheelRadiusMeters * Math.PI) / (double) 1024);

        // Set whether drive encoder should be reversed or not
        m_driveEncoder.setReverseDirection(driveEncoderReversed);

        // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        m_turningEncoder.setDistancePerPulse((2 * Math.PI) / (double) 1024);

        // Set whether turning encoder should be reversed or not
        m_turningEncoder.setReverseDirection(turningEncoderReversed);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
                SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
                m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
                m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        m_driveMotor.set(driveOutput);
        m_turningMotor.set(turnOutput);
    }

    public void setDesiredState(double speedMetersPerSecond, Rotation2d angle) {
        setDesiredState(new SwerveModuleState(speedMetersPerSecond, angle));
    }

    /**
     * Zeroes all the SwerveModule encoders.
     */
    public void resetEncoders() {
        m_driveEncoder.reset();
        m_turningEncoder.reset();
    }
}