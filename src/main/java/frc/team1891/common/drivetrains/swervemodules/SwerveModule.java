// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains.swervemodules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveModule {
    public static double degreesToMotorEncoderTicks(double degrees, double steeringGearRatio, double motorEncoderTicksPerRevolution) {
        return degrees / (360.0 / (steeringGearRatio * motorEncoderTicksPerRevolution));
    }

    public static double radiansToEncoderTicks(double radians, double steeringGearRatio, double motorEncoderTicksPerRevolution) {
        return radians / ((2*Math.PI) / (steeringGearRatio * motorEncoderTicksPerRevolution));
    }

    public static double motorEncoderTicksToDegrees(double motorEncoderTicks, double steeringGearRatio, double motorEncoderTicksPerRevolution) {
//        return Rotation2d.fromDegrees(bsfSwerveModule.Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), 150/7.));
        return motorEncoderTicks * (360.0 / (steeringGearRatio * motorEncoderTicksPerRevolution));
    }

    public static double motorEncoderTicksToRadians(double motorEncoderTicks, double steeringGearRatio, double motorEncoderTicksPerRevolution) {
//        return Rotation2d.fromDegrees(bsfSwerveModule.Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), 150/7.));
        return motorEncoderTicks * ((2*Math.PI) / (steeringGearRatio * motorEncoderTicksPerRevolution));
    }

    private final DriveController driveController;
    private final SteerController steerController;
    private SwerveModuleState desiredState;

    public SwerveModule(DriveController driveController, SteerController steerController) {
        this.driveController = driveController;
        this.steerController = steerController;
        this.desiredState = new SwerveModuleState();
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getWheelPosition(), getAngleRotation2d());
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getWheelVelocity(), getAngleRotation2d());
    }

    public void setDesiredSwerveModuleState(SwerveModuleState desiredState) {
        desiredState.angle = new Rotation2d(MathUtil.inputModulus(desiredState.angle.getRadians(), 0, 2*Math.PI));
        this.desiredState = desiredState;
        desiredState = SwerveModuleState.optimize(desiredState, getAngleRotation2d());
        drive(desiredState);
    }

    public SwerveModuleState getDesiredSwerveModuleState() {
        return desiredState;
    }

    /**
     * Drives the module to the desired state.
     * @param desiredState desired {@link SwerveModuleState}
     */
    private void drive(SwerveModuleState desiredState) {
        driveController.drive(desiredState);
        steerController.drive(desiredState);
    }

    /**
     * Stops the swerve module.
     */
    public void stop() {
        desiredState = new SwerveModuleState(0, getAngleRotation2d());
        driveController.stop();
        steerController.stop();
    }

    /**
     * Returns the position of the wheel in meters.
     * @return the position of the wheel
     */
    protected double getWheelPosition() {
        return driveController.getPosition();
    }

    /**
     * Returns the velocity of the wheel in meters per second.
     * @return the velocity of the wheel
     */
    protected double getWheelVelocity() {
        return driveController.getVelocity();
    }

    /**
     * Returns the angle of the module as a {@link Rotation2d}.
     * @return the angle of the module
     */
    public Rotation2d getAngleRotation2d() {
        return steerController.getRotation2d();
    }

    /**
     * Returns the angle of the module in radians.
     * @return the angle of the module
     */
    public double getAngleRadians() {
        return steerController.getRadians();
    }

    /**
     * Returns the angle of the module in degrees.
     * @return the angle of the module
     */
    public double getAngleDegrees() {
        return steerController.getDegrees();
    }

    /**
     * Configure the module to output its information to Shuffleboard
     * @param moduleLayout the layout for the module
     */
    public void configureShuffleboard(ShuffleboardLayout moduleLayout) {
        moduleLayout.addNumber("Current Position (Meters)", () -> getSwerveModulePosition().distanceMeters);
        moduleLayout.addNumber("Current Velocity (Meters per Second)", () -> getSwerveModuleState().speedMetersPerSecond);
        moduleLayout.addNumber("Current Angle (Radians)", this::getAngleRadians);
        moduleLayout.addNumber("Desired Velocity (Meters per Second)", () -> desiredState.speedMetersPerSecond);
        moduleLayout.addNumber("Desired Angle (Radians)", () -> desiredState.angle.getRadians());
    }
}