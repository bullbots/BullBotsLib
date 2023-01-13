// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public abstract class SwerveModule {
    public static SwerveModule createFromDriveFalconAndSteerFalcon(WPI_TalonFX driveFalcon,
                                                                   WPI_TalonFX steeringFalcon,
                                                                   CANCoder encoder,
                                                                   DrivetrainConfig config,
                                                                   double driveP,
                                                                   double driveI,
                                                                   double driveD,
                                                                   double steerP,
                                                                   double steerI,
                                                                   double steerD) {
        return new SwerveModule(encoder, driveP, driveI, driveD, steerP, steerI, steerD) {
            @Override
            public SwerveModuleState getState() {
                return new SwerveModuleState(driveFalcon.getSelectedSensorVelocity(), getCANCoderRotation2d());
            }

            @Override
            public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(driveFalcon.getSelectedSensorPosition(), getCANCoderRotation2d());
            }

            @Override
            public void setDesiredState(SwerveModuleState desiredState) {
                // Optimize the reference state to avoid spinning further than 90 degrees
                SwerveModuleState state =
                        SwerveModuleState.optimize(desiredState, getCANCoderRotation2d());

                // Calculate the drive output from the drive PID controller.
                final double driveOutput =
                        drivePIDController.calculate(config.nativeUnitsToVelocityMeters(driveFalcon.getSelectedSensorVelocity()), state.speedMetersPerSecond);

                // Calculate the turning motor output from the turning PID controller.
                final double turnOutput =
                        turningPIDController.calculate(getAbsoluteCANCoderRadians(), state.angle.getRadians());

                // Calculate the turning motor output from the turning PID controller.
                driveFalcon.set(driveOutput);
                steeringFalcon.set(turnOutput);
            }
        };
    }

    public static SwerveModule createFromDriveFalconAndSteeringNeo(WPI_TalonFX driveFalcon,
                                                                   CANSparkMax steeringNeo,
                                                                   CANCoder encoder,
                                                                   DrivetrainConfig config,
                                                                   double driveP,
                                                                   double driveI,
                                                                   double driveD,
                                                                   double steerP,
                                                                   double steerI,
                                                                   double steerD) {
        return new SwerveModule(encoder, driveP, driveI, driveD, steerP, steerI, steerD) {
            @Override
            public SwerveModuleState getState() {
                return new SwerveModuleState(driveFalcon.getSelectedSensorVelocity(), getCANCoderRotation2d());
            }

            @Override
            public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(driveFalcon.getSelectedSensorPosition(), getCANCoderRotation2d());
            }

            @Override
            public void setDesiredState(SwerveModuleState desiredState) {
                // Optimize the reference state to avoid spinning further than 90 degrees
                SwerveModuleState state =
                        SwerveModuleState.optimize(desiredState, getCANCoderRotation2d());

                // Calculate the drive output from the drive PID controller.
                final double driveOutput =
                        drivePIDController.calculate(config.nativeUnitsToVelocityMeters(driveFalcon.getSelectedSensorVelocity()), state.speedMetersPerSecond);

                // Calculate the turning motor output from the turning PID controller.
                final double turnOutput =
                        turningPIDController.calculate(getAbsoluteCANCoderRadians(), state.angle.getRadians());

                // Calculate the turning motor output from the turning PID controller.
                driveFalcon.set(driveOutput);
                steeringNeo.set(turnOutput);
            }
        };
    }

    private CANCoder encoder;

    protected PIDController drivePIDController;

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    protected ProfiledPIDController turningPIDController;

    private SwerveModule(CANCoder encoder, double driveP, double driveI, double driveD, double steerP, double steerI, double steerD) {
        this.encoder = encoder;

        drivePIDController =
                new PIDController(driveP, driveI, driveD);
        turningPIDController =
                new ProfiledPIDController(
                        steerP, steerI, steerD,
                        new TrapezoidProfile.Constraints(
                                // Max angular velocity and acceleration of the module
                                2*Math.PI,
                                2*Math.PI
                        )
                );
        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public abstract SwerveModuleState getState();

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public abstract SwerveModulePosition getPosition();

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public abstract void setDesiredState(SwerveModuleState desiredState);

    public void setDesiredState(double speedMetersPerSecond, Rotation2d angle) {
        setDesiredState(new SwerveModuleState(speedMetersPerSecond, angle));
    }

    public double getAbsoluteCANCoderRadians() {
        double angle = Math.toRadians(encoder.getAbsolutePosition());
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public Rotation2d getCANCoderRotation2d() {
        return new Rotation2d(getAbsoluteCANCoderRadians());
    }
}