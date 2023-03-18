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
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.hardware.NavX;

/** Drivetrain base for a differential drivetrain. */
@SuppressWarnings("unused")
public abstract class DifferentialDrivetrain extends Drivetrain {
  protected final DifferentialDriveKinematics kinematics;
  protected final DifferentialDriveOdometry odometry;

  protected final DifferentialDrive differentialDrive;

  protected final WPI_TalonFX left, right;

  /**
   * Creates a new differential drivetrain.
   * @param config the drivetrain config
   * @param kinematics the kinematics for this drivetrain
   * @param odometry the odometry for this drivetrain
   * @param gyro the gyro used by the drivetrain ({@link NavX})
   * @param leftMaster the left drive motor
   * @param rightMaster the right drive motor
   */
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

  /**
   * Drives the robot using {@link DifferentialDrive#arcadeDrive(double, double, boolean)}.
   * @param xSpeed forward speed
   * @param rot turning speed
   * @param squaredInputs square joystick inputs for more controlled movements at low speed
   */
  public void arcadeDrive(double xSpeed, double rot, boolean squaredInputs) {
    differentialDrive.arcadeDrive(xSpeed, rot, squaredInputs);
  }

  /**
   * Drives the robot using {@link DifferentialDrive#curvatureDrive(double, double, boolean)}.
   * @param xSpeed forward speed
   * @param zRotation turning speed
   * @param allowTurnInPlace allow the robot to turn if there is no forward input
   */
  public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace) {
    differentialDrive.curvatureDrive(xSpeed, zRotation, allowTurnInPlace);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /**
   * Returns the speeds of the left and right sides as a {@link DifferentialDriveWheelSpeeds}.
   * @return the speeds of the wheels
   */
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

  /**
   * Resets the encoders of the left and right motors
   */
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
    super.configureSmartDashboard();
    LazyDashboard.addNumber("Drivetrain/leftPosition", left::getSelectedSensorPosition);
    LazyDashboard.addNumber("Drivetrain/leftVelocity", left::getSelectedSensorVelocity);
    LazyDashboard.addNumber("Drivetrain/rightPosition", right::getSelectedSensorPosition);
    LazyDashboard.addNumber("Drivetrain/rightVelocity", right::getSelectedSensorVelocity);
    LazyDashboard.addNumber("Drivetrain/gyroAngle", gyro::getAngle);
    LazyDashboard.addNumber("Drivetrain/xSpeed (Meters per Second)", () -> getChassisSpeeds().vxMetersPerSecond);
    LazyDashboard.addNumber("Drivetrain/ySpeed (Meters per Second)", () -> getChassisSpeeds().vyMetersPerSecond);
    LazyDashboard.addNumber("Drivetrain/omegaSpeed (Radians per Second)", () -> getChassisSpeeds().omegaRadiansPerSecond);
  }
}
