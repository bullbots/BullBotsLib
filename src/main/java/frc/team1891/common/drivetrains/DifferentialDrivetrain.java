// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.hardware.NavX;

public class DifferentialDrivetrain extends Drivetrain {
  protected final DifferentialDriveKinematics kinematics;
  protected final DifferentialDriveOdometry odometry;

  protected final DifferentialDrive differentialDrive;

  protected final WPI_TalonFX left, right;

  public DifferentialDrivetrain(
    ShuffleboardTab shuffleboardTab,
    DrivetrainConfig config,
    DifferentialDriveKinematics kinematics,
    DifferentialDriveOdometry odometry,
    NavX gyro,
    WPI_TalonFX leftMaster,
    WPI_TalonFX rightMaster
  ) {
    super(shuffleboardTab, config, gyro);

    this.kinematics = kinematics;
    this.odometry = odometry;

    differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
    differentialDrive.setSafetyEnabled(false);

    this.left = leftMaster;
    this.right = rightMaster;
  }

  public void arcadeDrive(double xSpeed, double rot, boolean squaredInputs) {
    differentialDrive.arcadeDrive(xSpeed, rot, squaredInputs);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace) {
    differentialDrive.curvatureDrive(xSpeed, zRotation, allowTurnInPlace);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left.getSelectedSensorVelocity(), right.getSelectedSensorVelocity());
  }

  @Override
  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  @Override
  public void stop() {
    differentialDrive.stopMotor();
  }

  @Override
  public void resetOdometry(Pose2d pose2d) {
    resetEncoders();
    odometry.resetPosition(gyro.getRotation2d(), left.getSelectedSensorPosition(), right.getSelectedSensorPosition(), pose2d);
  }

  public void resetEncoders() {
    left.setSelectedSensorPosition(0);
    right.setSelectedSensorPosition(0);
  }

  @Override
  public void updateOdometry() {
    odometry.update(gyro.getRotation2d(), left.getSelectedSensorPosition(), right.getSelectedSensorPosition());
  }

  @Override
  protected void configureSmartDashboard() {
    LazyDashboard.addNumber("Drivetrain/leftPosition", left::getSelectedSensorPosition);
    LazyDashboard.addNumber("Drivetrain/leftVelocity", left::getSelectedSensorVelocity);
    LazyDashboard.addNumber("Drivetrain/rightPosition", right::getSelectedSensorPosition);
    LazyDashboard.addNumber("Drivetrain/rightVelocity", right::getSelectedSensorVelocity);
    LazyDashboard.addNumber("Drivetrain/gyroRadians", gyro::getRadians);
    LazyDashboard.addNumber("Drivetrain/gyroDegrees", gyro::getDegrees);
    LazyDashboard.addNumber("Drivetrain/gyroDegreesLooped", gyro::getDegreesLooped);
    LazyDashboard.addNumber("Drivetrain/xSpeed (Meters per Second)", () -> getChassisSpeeds().vxMetersPerSecond);
    LazyDashboard.addNumber("Drivetrain/ySpeed (Meters per Second)", () -> getChassisSpeeds().vyMetersPerSecond);
    LazyDashboard.addNumber("Drivetrain/omegaSpeed (Radians per Second)", () -> getChassisSpeeds().omegaRadiansPerSecond);
  }
}
