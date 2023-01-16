// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team1891.common.hardware.NavX;

/** Mecanum Drivetrain base. */
public class MecanumDrivetrain extends HolonomicDrivetrain {

  protected final MecanumDriveOdometry odometry;
  protected final MecanumDriveKinematics kinematics;

  private final TalonFX frontLeft, frontRight, backLeft, backRight;

  public MecanumDrivetrain(
    ShuffleboardTab shuffleboardTab,
    DrivetrainConfig config,
    MecanumDriveKinematics kinematics,
    MecanumDriveOdometry odometry,
    NavX gyro,
    TalonFX frontLeft,
    TalonFX frontRight,
    TalonFX backLeft,
    TalonFX backRight
  ) {
    super(shuffleboardTab, config, gyro);

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

  @Override
  protected void configureShuffleboard() {
    ShuffleboardLayout frontLeftLayout = shuffleboardTab.getLayout("Front Left", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    frontLeftLayout.addNumber("Position", frontLeft::getSelectedSensorPosition);
    frontLeftLayout.addNumber("Velocity", frontLeft::getSelectedSensorVelocity);
    ShuffleboardLayout frontRightLayout = shuffleboardTab.getLayout("Front Right", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
    frontRightLayout.addNumber("Position", frontRight::getSelectedSensorPosition);
    frontRightLayout.addNumber("Velocity", frontRight::getSelectedSensorVelocity);
    ShuffleboardLayout backLeftLayout = shuffleboardTab.getLayout("Back Left", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0);
    backLeftLayout.addNumber("Position", backLeft::getSelectedSensorPosition);
    backLeftLayout.addNumber("Velocity", backLeft::getSelectedSensorVelocity);
    ShuffleboardLayout backRightLayout = shuffleboardTab.getLayout("Back Right", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0);
    backRightLayout.addNumber("Position", backRight::getSelectedSensorPosition);
    backRightLayout.addNumber("Velocity", backRight::getSelectedSensorVelocity);
    ShuffleboardLayout gyroLayout = shuffleboardTab.getLayout("Gyro", BuiltInLayouts.kList).withSize(2, 3).withPosition(0, 4);
    gyroLayout.addNumber("Radians", gyro::getRadians);
    gyroLayout.addNumber("Degrees", gyro::getDegrees);
    gyroLayout.addNumber("Degrees (Looped)", gyro::getDegreesLooped);
    shuffleboardTab.addNumber("Chassis x Speed (Meters per Second)", () -> kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(config.nativeUnitsToVelocityMeters(frontLeft.getSelectedSensorVelocity()), config.nativeUnitsToVelocityMeters(frontRight.getSelectedSensorVelocity()), config.nativeUnitsToVelocityMeters(backLeft.getSelectedSensorVelocity()), config.nativeUnitsToVelocityMeters(backRight.getSelectedSensorVelocity()))).vxMetersPerSecond);
    shuffleboardTab.addNumber("Chassis y Speed (Meters per Second)", () -> kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(config.nativeUnitsToVelocityMeters(frontLeft.getSelectedSensorVelocity()), config.nativeUnitsToVelocityMeters(frontRight.getSelectedSensorVelocity()), config.nativeUnitsToVelocityMeters(backLeft.getSelectedSensorVelocity()), config.nativeUnitsToVelocityMeters(backRight.getSelectedSensorVelocity()))).vxMetersPerSecond);
    shuffleboardTab.addNumber("Chassis ω Speed (Radians per Second)", () -> kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(config.nativeUnitsToVelocityMeters(frontLeft.getSelectedSensorVelocity()), config.nativeUnitsToVelocityMeters(frontRight.getSelectedSensorVelocity()), config.nativeUnitsToVelocityMeters(backLeft.getSelectedSensorVelocity()), config.nativeUnitsToVelocityMeters(backRight.getSelectedSensorVelocity()))).vxMetersPerSecond);
  }
}
