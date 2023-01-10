// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.*;
import frc.team1891.common.hardware.NavX;

/** Mecanum Drivetrain base. */
public class MecanumDrivetrain extends HolonomicDrivetrain {

  protected final MecanumDriveOdometry odometry;
  protected final MecanumDriveKinematics kinematics;

  private final TalonFX frontLeft, frontRight, backLeft, backRight;

  public MecanumDrivetrain(
    DrivetrainConfig config,
    MecanumDriveKinematics kinematics,
    MecanumDriveOdometry odometry,
    NavX gyro,
    TalonFX frontLeft,
    TalonFX frontRight,
    TalonFX backLeft,
    TalonFX backRight
  ) {
    super(config, gyro);

    this.odometry = odometry;
    this.kinematics = kinematics;

    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;
  }

  public void holonomicDrive(double xSpeed, double ySpeed, double rot, boolean fieldOriented) {
    xSpeed *= config.chassisMaxVelocityMetersPerSecond;
    ySpeed *= config.chassisMaxVelocityMetersPerSecond;
    rot *= config.chassisMaxAngularVelocityRadiansPerSecond;

    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(
      fieldOriented?
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
      :
        new ChassisSpeeds(xSpeed, ySpeed, rot)
    );

    setWheelSpeeds(wheelSpeeds);
  }

  @Override
  public void fromChassisSpeeds(ChassisSpeeds speeds) {
    setWheelSpeeds(kinematics.toWheelSpeeds(speeds));
  }

  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    wheelSpeeds.desaturate(config.chassisMaxVelocityMetersPerSecond);

    frontLeft.set(ControlMode.PercentOutput, wheelSpeeds.frontLeftMetersPerSecond / config.chassisMaxVelocityMetersPerSecond);
    frontRight.set(ControlMode.PercentOutput, wheelSpeeds.frontRightMetersPerSecond / config.chassisMaxVelocityMetersPerSecond);
    backLeft.set(ControlMode.PercentOutput, wheelSpeeds.rearLeftMetersPerSecond / config.chassisMaxVelocityMetersPerSecond);
    backRight.set(ControlMode.PercentOutput, wheelSpeeds.rearRightMetersPerSecond / config.chassisMaxVelocityMetersPerSecond);
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      config.nativeUnitsToVelocityMeters(frontLeft.getSelectedSensorVelocity()),
      config.nativeUnitsToVelocityMeters(frontRight.getSelectedSensorVelocity()),
      config.nativeUnitsToVelocityMeters(backLeft.getSelectedSensorVelocity()),
      config.nativeUnitsToVelocityMeters(backRight.getSelectedSensorVelocity())
    );
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
      config.nativeUnitsToDistanceMeters(frontLeft.getSelectedSensorPosition()),
      config.nativeUnitsToDistanceMeters(frontRight.getSelectedSensorPosition()),
      config.nativeUnitsToDistanceMeters(backLeft.getSelectedSensorPosition()),
      config.nativeUnitsToDistanceMeters(backRight.getSelectedSensorPosition())
    );
  }

  public MecanumDriveKinematics getKinematics() {
      return kinematics;
  }

  @Override
  public void stop() {
    frontLeft.set(ControlMode.Disabled, 0);
    frontRight.set(ControlMode.Disabled, 0);
    backLeft.set(ControlMode.Disabled, 0);
    backRight.set(ControlMode.Disabled, 0);
  }

  @Override
  public void updateOdometry() {
    odometry.update(gyro.getRotation2d(), getWheelPositions());
  }

  @Override
  public Pose2d getPose2d() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void resetOdometry(Pose2d pose2d) {
    // TODO Auto-generated method stub
    
  }
}
