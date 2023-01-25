// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public abstract class SwerveModule {
    public static SwerveModule createFromDriveFalconAndSteerFalcon(WPI_TalonFX driveFalcon,
                                                                   WPI_TalonFX steerFalcon,
                                                                   WPI_CANCoder encoder,
                                                                   DrivetrainConfig config,
                                                                   double driveP,
                                                                   double driveI,
                                                                   double driveD,
                                                                   double steerP,
                                                                   double steerI,
                                                                   double steerD,
                                                                   double driveFeedForwardV) {
        return new SwerveModule(encoder, driveP, driveI, driveD, steerP, steerI, steerD, driveFeedForwardV) {
            @Override
            public SwerveModuleState getState() {
                return new SwerveModuleState(config.encoderTicksPer100msToVelocity(driveFalcon.getSelectedSensorVelocity()), getCANCoderRotation2d());
            }

            @Override
            public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(config.encoderTicksToDistance(driveFalcon.getSelectedSensorPosition()), getCANCoderRotation2d());
            }
//
//            @Override
//            public void drive(SwerveModuleState desiredState) {
////                // Optimize the reference state to avoid spinning further than 90 degrees
////                SwerveModuleState state =
////                        SwerveModuleState.optimize(desiredState, getCANCoderRotation2d());
//
//                // Calculate the drive output from the drive PID controller.
//                final double driveOutput =
//                        drivePIDController.calculate(config.nativeUnitsToVelocityMeters(driveFalcon.getSelectedSensorVelocity()), desiredState.speedMetersPerSecond);
//
//                // Calculate the turning motor output from the turning PID controller.
//                final double turnOutput =
//                        turningPIDController.calculate(getAbsoluteCANCoderRadians(), desiredState.angle.getRadians());
//
//                // Calculate the turning motor output from the turning PID controller.
//                driveFalcon.set(driveOutput);
//                steerFalcon.set(turnOutput);
//            }


//            @Override
//            protected void drive(SwerveModuleState desiredState) {
//                driveFalcon.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond / config.chassisMaxVelocityMetersPerSecond);
//                // Calculate the turning motor output from the turning PID controller.
//                final double turnOutput =
//                        turningPIDController.calculate(getAbsoluteCANCoderRadians(), desiredState.angle.getRadians());
//                steerFalcon.set(turnOutput);
//            }

            protected void drive(SwerveModuleState desiredState) {
                double velocity = config.velocityToEncoderTicksPer100ms(desiredState.speedMetersPerSecond);
                driveFalcon.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, driveFeedForwardController.calculate(desiredState.speedMetersPerSecond));

                // Not sure why, but this doesn't seem to have any effect - at least not in the sim
//                steerFalcon.set(ControlMode.Position, desiredState.angle.getRadians() / ((2*Math.PI)/((150/7.)*2048)));

                // Calculate the turning motor output from the turning PID controller.
                final double turnOutput =
                        turningPIDController.calculate(getAbsoluteCANCoderRadians(), desiredState.angle.getRadians());
                steerFalcon.set(turnOutput);
            }

            @Override
            public void stop() {
                setDesiredState(getState());
                driveFalcon.stopMotor();
                steerFalcon.stopMotor();
            }
        };
    }

    public static SwerveModule createFromDriveFalconAndSteerNeo(WPI_TalonFX driveFalcon,
                                                                   CANSparkMax steerNeo,
                                                                   WPI_CANCoder encoder,
                                                                   DrivetrainConfig config,
                                                                   double driveP,
                                                                   double driveI,
                                                                   double driveD,
                                                                   double steerP,
                                                                   double steerI,
                                                                   double steerD,
                                                                   double driveFeedForwardV) {
        return new SwerveModule(encoder, driveP, driveI, driveD, steerP, steerI, steerD, driveFeedForwardV) {
            @Override
            public SwerveModuleState getState() {
                return new SwerveModuleState(config.encoderTicksPer100msToVelocity(driveFalcon.getSelectedSensorVelocity()), getCANCoderRotation2d());
            }

            @Override
            public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(config.encoderTicksToDistance(driveFalcon.getSelectedSensorPosition()), getCANCoderRotation2d());
            }

//            @Override
//            public void drive(SwerveModuleState desiredState) {
////                // Optimize the reference state to avoid spinning further than 90 degrees
////                SwerveModuleState state =
////                        SwerveModuleState.optimize(desiredState, getCANCoderRotation2d());
//
//                // Calculate the drive output from the drive PID controller.
//                final double driveOutput =
//                        drivePIDController.calculate(config.encoderTicksPer100msToVelocity(driveFalcon.getSelectedSensorVelocity()), desiredState.speedMetersPerSecond);
//
//                // Calculate the turning motor output from the turning PID controller.
//                final double turnOutput =
//                        turningPIDController.calculate(getAbsoluteCANCoderRadians(), desiredState.angle.getRadians());
//
//                // Calculate the turning motor output from the turning PID controller.
//                driveFalcon.set(driveOutput);
//                steerNeo.set(turnOutput);
//            }

            @Override
            protected void drive(SwerveModuleState desiredState) {
                double velocity = config.velocityToEncoderTicksPer100ms(desiredState.speedMetersPerSecond);
                driveFalcon.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, driveFeedForwardController.calculate(desiredState.speedMetersPerSecond));


                // Calculate the turning motor output from the turning PID controller.
                final double turnOutput =
                        turningPIDController.calculate(getAbsoluteCANCoderRadians(), desiredState.angle.getRadians());
                steerNeo.set(turnOutput);
            }

            @Override
            public void stop() {
                setDesiredState(getState());
                driveFalcon.stopMotor();
                steerNeo.stopMotor();
            }
        };
    }

    public static SwerveModule createFromDriveFalconAndSteerNeo(WPI_TalonFX driveFalcon,
                                                                CANSparkMax steerNeo,
                                                                WPI_CANCoder encoder,
                                                                DrivetrainConfig config,
                                                                double driveP,
                                                                double driveI,
                                                                double driveD,
                                                                double driveFeedForwardV) {
        return new SwerveModule(encoder, driveP, driveI, driveD, 0, 0, 0, driveFeedForwardV) {
            SparkMaxPIDController internalPIDController = steerNeo.getPIDController();
            @Override
            public SwerveModuleState getState() {
                return new SwerveModuleState(config.encoderTicksPer100msToVelocity(driveFalcon.getSelectedSensorVelocity()), getCANCoderRotation2d());
            }

            @Override
            public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(config.encoderTicksToDistance(driveFalcon.getSelectedSensorPosition()), getCANCoderRotation2d());
            }

//            @Override
//            public void drive(SwerveModuleState desiredState) {
////                // Optimize the reference state to avoid spinning further than 90 degrees
////                SwerveModuleState state =
////                        SwerveModuleState.optimize(desiredState, getCANCoderRotation2d());
//
//                // Calculate the drive output from the drive PID controller.
//                final double driveOutput =
//                        drivePIDController.calculate(config.encoderTicksPer100msToVelocity(driveFalcon.getSelectedSensorVelocity()), desiredState.speedMetersPerSecond);
//
//                // Calculate the turning motor output from the turning PID controller.
//                final double turnOutput =
//                        turningPIDController.calculate(getAbsoluteCANCoderRadians(), desiredState.angle.getRadians());
//
//                // Calculate the turning motor output from the turning PID controller.
//                driveFalcon.set(driveOutput);
//                steerNeo.set(turnOutput);
//            }

            @Override
            protected void drive(SwerveModuleState desiredState) {
                double velocity = config.velocityToEncoderTicksPer100ms(desiredState.speedMetersPerSecond);
                driveFalcon.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, driveFeedForwardController.calculate(desiredState.speedMetersPerSecond));


                // Calculate the turning motor output from the turning PID controller.
                final double turnOutput =
                        turningPIDController.calculate(getAbsoluteCANCoderRadians(), desiredState.angle.getRadians());
                steerNeo.set(turnOutput);
            }

            @Override
            public void stop() {
                setDesiredState(getState());
                driveFalcon.stopMotor();
                steerNeo.stopMotor();
            }
        };
    }

    private final WPI_CANCoder encoder;

    private SwerveModuleState desiredState = new SwerveModuleState();

    protected PIDController drivePIDController;
    protected SimpleMotorFeedforward driveFeedForwardController;

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    protected ProfiledPIDController turningPIDController;

    private SwerveModule(WPI_CANCoder encoder, double driveP, double driveI, double driveD, double steerP, double steerI, double steerD, double driveFeedForwardV) {
        this.encoder = encoder;

        drivePIDController = new PIDController(driveP, driveI, driveD);
        driveFeedForwardController = new SimpleMotorFeedforward(0, driveFeedForwardV, 0);

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

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public abstract SwerveModulePosition getPosition();

    /**
     * Drives the drive and steer motors to the desired state.
     * @param desiredState the already optimized state to drive to
     */
    protected abstract void drive(SwerveModuleState desiredState);

    /**
     * Stops the drive and steer motors.
     */
    public abstract void stop();

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.angle = new Rotation2d(MathUtil.inputModulus(desiredState.angle.getRadians(), 0, 2*Math.PI));
        this.desiredState = desiredState;
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getCANCoderRotation2d());
        drive(optimizedState);
//        this.desiredState = optimizedState;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param speedMetersPerSecond desired speed of the wheel
     * @param angle desired angle of the module
     */
    public void setDesiredState(double speedMetersPerSecond, Rotation2d angle) {
        setDesiredState(new SwerveModuleState(speedMetersPerSecond, angle));
    }

    /**
     * Get the position of the module as read by the CANCoder.
     * @return the CANCoder position in radians in the range (0, 2pi]
     */
    public double getAbsoluteCANCoderRadians() {
        return MathUtil.inputModulus(Math.toRadians(encoder.getAbsolutePosition()), 0, 2*Math.PI);
    }

    /**
     * Get the position of the module as read by the CANCoder, as a {@link Rotation2d}.
     * @return the CANCoder position
     */
    public Rotation2d getCANCoderRotation2d() {
        return new Rotation2d(getAbsoluteCANCoderRadians());
    }

    /**
     * Configure the module to output it's information to Shuffleboard
     * @param moduleLayout the layout for the module
     */
    public void configureShuffleboard(ShuffleboardLayout moduleLayout) {
        moduleLayout.addNumber("Current Position (Meters)", () -> getPosition().distanceMeters);
        moduleLayout.addNumber("Current Velocity (Meters per Second)", () -> getState().speedMetersPerSecond);
        moduleLayout.addNumber("Current Angle (Radians)", this::getAbsoluteCANCoderRadians);
        moduleLayout.addNumber("Desired Velocity (Meters per Second)", () -> desiredState.speedMetersPerSecond);
        moduleLayout.addNumber("Desired Angle (Radians)", () -> desiredState.angle.getRadians());
    }
}