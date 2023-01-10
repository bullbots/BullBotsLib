// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team1891.common.hardware.NavX;

public class DifferentialDrivetrain extends Drivetrain {
  protected final DifferentialDriveKinematics kinematics;
  protected final DifferentialDriveOdometry odometry;

  protected final DifferentialDrive differentialDrive;

  protected final WPI_TalonFX left, right;

  public DifferentialDrivetrain(
    DrivetrainConfig config,
    DifferentialDriveKinematics kinematics,
    DifferentialDriveOdometry odometry,
    NavX gyro,
    WPI_TalonFX leftMaster,
    WPI_TalonFX rightMaster
  ) {
    super(config, gyro);

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
}
