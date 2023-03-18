// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.hardware.NavX;

/** Drivetrain base for a mecanum drivetrain. */
@SuppressWarnings("unused")
public abstract class MecanumDrivetrain extends HolonomicDrivetrain {
  protected final MecanumDrivePoseEstimator poseEstimator;
  protected final MecanumDriveKinematics kinematics;

  private final WPI_TalonFX frontLeft, frontRight, backLeft, backRight;

  /**
   * Creates a new MecanumDrivetrain, assuming the center of the robot is the center of the drivebase.
   * @param config the config of the drivetrain
   * @param driveBaseWidth the width between left and right wheels
   * @param driveBaseLength the length between front and back wheels
   * @param gyro the gyro of the drivetrain
   * @param frontLeft the front left wheels
   * @param frontRight the front right wheels
   * @param backLeft the back left wheels
   * @param backRight the back right wheels
   */
  public MecanumDrivetrain(
    DrivetrainConfig config,
    double driveBaseWidth,
    double driveBaseLength,
    NavX gyro,
    WPI_TalonFX frontLeft,
    WPI_TalonFX frontRight,
    WPI_TalonFX backLeft,
    WPI_TalonFX backRight
  ) {
    this(
      config,
      new Translation2d(driveBaseLength / 2d, driveBaseWidth / 2d),
      new Translation2d(driveBaseLength / 2d, -driveBaseWidth / 2d),
      new Translation2d(-driveBaseLength / 2d, driveBaseWidth / 2d),
      new Translation2d(-driveBaseLength / 2d, -driveBaseWidth / 2d),
      gyro,
      frontLeft,
      frontRight,
      backLeft,
      backRight
    );
  }

  /**
   * Creates a new MecanumDrivetrain.
   * @param config the config of the drivetrain
   * @param frontLeftLocation the location of the front left wheel relative to the robot center
   * @param frontRightLocation the location of the front right wheel relative to the robot center
   * @param backLeftLocation the location of the back left wheel relative to the robot center
   * @param backRightLocation the location of the back right wheel relative to the robot center
   * @param gyro the gyro of the drivetrain
   * @param frontLeft the front left wheel
   * @param frontRight the front right wheel
   * @param backLeft the back left wheel
   * @param backRight the back right wheel
   */
  public MecanumDrivetrain(
    DrivetrainConfig config,
    Translation2d frontLeftLocation,
    Translation2d frontRightLocation,
    Translation2d backLeftLocation,
    Translation2d backRightLocation,
    NavX gyro,
    WPI_TalonFX frontLeft,
    WPI_TalonFX frontRight,
    WPI_TalonFX backLeft,
    WPI_TalonFX backRight
  ) {
    super(config, gyro);

    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;

    this.kinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    this.poseEstimator = new MecanumDrivePoseEstimator(kinematics, gyro.getRotation2d(), getWheelPositions(), new Pose2d());
  }

  @Override
  public void holonomicDrive(double xSpeed, double ySpeed, double rot, boolean fieldOriented) {
    xSpeed *= config.chassisMaxVelocityMetersPerSecond();
    ySpeed *= config.chassisMaxVelocityMetersPerSecond();
    rot *= config.chassisMaxAngularVelocityRadiansPerSecond();

    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(
      fieldOriented?
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose2d().getRotation())
      :
        new ChassisSpeeds(xSpeed, ySpeed, rot)
    );

    setWheelSpeeds(wheelSpeeds);
  }

  @Override
  public void fromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setWheelSpeeds(kinematics.toWheelSpeeds(chassisSpeeds));
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /**
   * Sets the speed of the wheels
   * @param wheelSpeeds the desired wheel speeds
   */
  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    wheelSpeeds.desaturate(config.chassisMaxVelocityMetersPerSecond());

    // TODO: This could - and probably should be ControlMode.Velocity
    frontLeft.set(ControlMode.PercentOutput, wheelSpeeds.frontLeftMetersPerSecond / config.chassisMaxVelocityMetersPerSecond());
    frontRight.set(ControlMode.PercentOutput, wheelSpeeds.frontRightMetersPerSecond / config.chassisMaxVelocityMetersPerSecond());
    backLeft.set(ControlMode.PercentOutput, wheelSpeeds.rearLeftMetersPerSecond / config.chassisMaxVelocityMetersPerSecond());
    backRight.set(ControlMode.PercentOutput, wheelSpeeds.rearRightMetersPerSecond / config.chassisMaxVelocityMetersPerSecond());
  }

  /**
   * Returns the speeds of the left and right sides as a {@link MecanumDriveWheelSpeeds}.
   * @return the speeds of the wheels
   */
  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      config.encoderTicksPer100msToVelocity(frontLeft.getSelectedSensorVelocity()),
      config.encoderTicksPer100msToVelocity(frontRight.getSelectedSensorVelocity()),
      config.encoderTicksPer100msToVelocity(backLeft.getSelectedSensorVelocity()),
      config.encoderTicksPer100msToVelocity(backRight.getSelectedSensorVelocity())
    );
  }

  /**
   * Returns the positions of the left and right sides as a {@link MecanumDriveWheelPositions}.
   * @return the positions of the wheels
   */
  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
      config.encoderTicksToDistance(frontLeft.getSelectedSensorPosition()),
      config.encoderTicksToDistance(frontRight.getSelectedSensorPosition()),
      config.encoderTicksToDistance(backLeft.getSelectedSensorPosition()),
      config.encoderTicksToDistance(backRight.getSelectedSensorPosition())
    );
  }

  /**
   * Returns the {@link MecanumDriveKinematics} of this drivetrain.
   * @return kinematics
   */
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
    poseEstimator.update(gyro.getRotation2d(), getWheelPositions());
  }

  @Override
  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void resetOdometry(Pose2d pose2d) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getWheelPositions(), pose2d);
  }

  @Override
  protected void configureSmartDashboard() {
    super.configureSmartDashboard();
    LazyDashboard.addNumber("Drivetrain/frontLeftPosition", frontLeft::getSelectedSensorPosition);
    LazyDashboard.addNumber("Drivetrain/frontLeftVelocity", frontLeft::getSelectedSensorVelocity);
    LazyDashboard.addNumber("Drivetrain/frontRightPosition", frontRight::getSelectedSensorPosition);
    LazyDashboard.addNumber("Drivetrain/frontRightVelocity", frontRight::getSelectedSensorVelocity);
    LazyDashboard.addNumber("Drivetrain/backLeftPosition", backLeft::getSelectedSensorPosition);
    LazyDashboard.addNumber("Drivetrain/backLeftVelocity", backLeft::getSelectedSensorVelocity);
    LazyDashboard.addNumber("Drivetrain/backRightPosition", backRight::getSelectedSensorPosition);
    LazyDashboard.addNumber("Drivetrain/backRightVelocity", backRight::getSelectedSensorVelocity);
    LazyDashboard.addNumber("Drivetrain/gyroRadians", gyro::getAngle);
    LazyDashboard.addNumber("Drivetrain/xSpeed (Meters per Second)", () -> getChassisSpeeds().vxMetersPerSecond);
    LazyDashboard.addNumber("Drivetrain/ySpeed (Meters per Second)", () -> getChassisSpeeds().vyMetersPerSecond);
    LazyDashboard.addNumber("Drivetrain/omegaSpeed (Radians per Second)", () -> getChassisSpeeds().omegaRadiansPerSecond);
  }
}
