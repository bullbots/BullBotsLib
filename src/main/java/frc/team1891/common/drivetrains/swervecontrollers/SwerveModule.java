// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains.swervecontrollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team1891.common.LazyDashboard;

@SuppressWarnings("unused")
public class SwerveModule {
    /**
     * Converts from the module angle in degrees to the motor position in encoder ticks.
     * @param degrees module angle in degrees
     * @param steeringGearRatio gear ratio between rotations of the module and the steering motor (motor rotations / module rotations)
     * @param motorEncoderTicksPerRevolution encoder counts of motor per revolution (e.g. TalonFX = 2048)
     * @return the position in encoder ticks of the steer motor
     */
    public static double degreesToMotorEncoderTicks(double degrees, double steeringGearRatio, double motorEncoderTicksPerRevolution) {
        return degrees / (360.0 / (steeringGearRatio * motorEncoderTicksPerRevolution));
    }

    /**
     * Converts from the module angle in radians to the motor position in encoder ticks.
     * @param radians module angle in radians
     * @param steeringGearRatio gear ratio between rotations of the module and the steering motor (motor rotations / module rotations)
     * @param motorEncoderTicksPerRevolution encoder counts of motor per revolution (e.g. TalonFX = 2048)
     * @return the position in encoder ticks of the steer motor
     */
    public static double radiansToEncoderTicks(double radians, double steeringGearRatio, double motorEncoderTicksPerRevolution) {
        return radians / ((2*Math.PI) / (steeringGearRatio * motorEncoderTicksPerRevolution));
    }

    /**
     * Converts from the motor position in encoder ticks to the module angle in degrees.
     * @param motorEncoderTicks the position in encoder ticks of the steer motor
     * @param steeringGearRatio gear ratio between rotations of the module and the steering motor (motor rotations / module rotations)
     * @param motorEncoderTicksPerRevolution encoder counts of motor per revolution (e.g. TalonFX = 2048)
     * @return the module angle in degrees
     */
    public static double motorEncoderTicksToDegrees(double motorEncoderTicks, double steeringGearRatio, double motorEncoderTicksPerRevolution) {
//        return Rotation2d.fromDegrees(bsfSwerveModule.Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), 150/7.));
        return motorEncoderTicks * (360.0 / (steeringGearRatio * motorEncoderTicksPerRevolution));
    }

    /**
     * Converts from the motor position in encoder ticks to the module angle in radians.
     * @param motorEncoderTicks the position in encoder ticks of the steer motor
     * @param steeringGearRatio gear ratio between rotations of the module and the steering motor (motor rotations / module rotations)
     * @param motorEncoderTicksPerRevolution encoder counts of motor per revolution (e.g. TalonFX = 2048)
     * @return the module angle in radians
     */
    public static double motorEncoderTicksToRadians(double motorEncoderTicks, double steeringGearRatio, double motorEncoderTicksPerRevolution) {
//        return Rotation2d.fromDegrees(bsfSwerveModule.Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), 150/7.));
        return motorEncoderTicks * ((2*Math.PI) / (steeringGearRatio * motorEncoderTicksPerRevolution));
    }

    private final DriveController driveController;
    private final SteerController steerController;
    private SwerveModuleState desiredState;

    /**
     * Creates a new swerve module using the given {@link DriveController} and {@link SteerController}.
     * @param driveController the drive controller
     * @param steerController the steer controller
     */
    public SwerveModule(DriveController driveController, SteerController steerController) {
        this.driveController = driveController;
        this.steerController = steerController;
        this.desiredState = new SwerveModuleState();
    }

    /**
     * Returns the current {@link SwerveModulePosition} of the module.
     * @return the current position of the module
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getWheelPosition(), getAngleRotation2d());
    }

    /**
     * Returns the current {@link SwerveModuleState} of the module.
     * @return the current state of the module
     */
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getWheelVelocity(), getAngleRotation2d());
    }

    /**
     * Sets the desired {@link SwerveModuleState} of the module and drives to it.
     * @param desiredState the desired state of the module
     */
    public void setDesiredSwerveModuleState(SwerveModuleState desiredState) {
        desiredState.angle = new Rotation2d(MathUtil.inputModulus(desiredState.angle.getRadians(), 0, 2*Math.PI));
        this.desiredState = desiredState;
        desiredState = SwerveModuleState.optimize(desiredState, getAngleRotation2d());
        drive(desiredState);
    }

    /**
     * Returns the desired {@link SwerveModuleState} of the module.
     * @return the desired state of the module
     */
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
     * Configure the module to output its information to SmartDashboard
     * @param moduleName the name for the module
     */
    public void configureSmartDashboard(String moduleName) {
        LazyDashboard.addNumber("Drivetrain/"+moduleName+"/Current Position (Meters)", () -> getSwerveModulePosition().distanceMeters);
        LazyDashboard.addNumber("Drivetrain/"+moduleName+"/Current Velocity (Meters per Second)", () -> getSwerveModuleState().speedMetersPerSecond);
        LazyDashboard.addNumber("Drivetrain/"+moduleName+"/Current Angle (Radians)", this::getAngleRadians);
        LazyDashboard.addNumber("Drivetrain/"+moduleName+"/Desired Velocity (Meters per Second)", () -> desiredState.speedMetersPerSecond);
        LazyDashboard.addNumber("Drivetrain/"+moduleName+"/Desired Angle (Radians)", () -> desiredState.angle.getRadians());
    }
}