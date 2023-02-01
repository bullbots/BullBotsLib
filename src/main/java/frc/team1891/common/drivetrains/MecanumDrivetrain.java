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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team1891.common.hardware.NavX;

/** Mecanum Drivetrain base. */
public class MecanumDrivetrain extends HolonomicDrivetrain {

  protected final MecanumDrivePoseEstimator poseEstimator;
  protected final MecanumDriveKinematics kinematics;

  private final WPI_TalonFX frontLeft, frontRight, backLeft, backRight;

  /**
   * Creates a new MecanumDrivetrain, assuming the center of the robot is the center of the drivebase.
   * @param shuffleboardTab the shuffleboard tab for the content of this subsystem
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
    ShuffleboardTab shuffleboardTab,
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
      shuffleboardTab,
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
   * @param shuffleboardTab the shuffleboard tab for the content of this subsystem
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
    ShuffleboardTab shuffleboardTab,
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
    super(shuffleboardTab, config, gyro);

    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;

    this.kinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    this.poseEstimator = new MecanumDrivePoseEstimator(kinematics, gyro.getRotation2d(), getWheelPositions(), new Pose2d());
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

    // TODO: This could - and probably should be ControlMode.Velocity
    frontLeft.set(ControlMode.PercentOutput, wheelSpeeds.frontLeftMetersPerSecond / config.chassisMaxVelocityMetersPerSecond);
    frontRight.set(ControlMode.PercentOutput, wheelSpeeds.frontRightMetersPerSecond / config.chassisMaxVelocityMetersPerSecond);
    backLeft.set(ControlMode.PercentOutput, wheelSpeeds.rearLeftMetersPerSecond / config.chassisMaxVelocityMetersPerSecond);
    backRight.set(ControlMode.PercentOutput, wheelSpeeds.rearRightMetersPerSecond / config.chassisMaxVelocityMetersPerSecond);
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      config.encoderTicksPer100msToVelocity(frontLeft.getSelectedSensorVelocity()),
      config.encoderTicksPer100msToVelocity(frontRight.getSelectedSensorVelocity()),
      config.encoderTicksPer100msToVelocity(backLeft.getSelectedSensorVelocity()),
      config.encoderTicksPer100msToVelocity(backRight.getSelectedSensorVelocity())
    );
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
      config.encoderTicksToDistance(frontLeft.getSelectedSensorPosition()),
      config.encoderTicksToDistance(frontRight.getSelectedSensorPosition()),
      config.encoderTicksToDistance(backLeft.getSelectedSensorPosition()),
      config.encoderTicksToDistance(backRight.getSelectedSensorPosition())
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
    shuffleboardTab.addNumber("Chassis x Speed (Meters per Second)", () -> kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(config.encoderTicksPer100msToVelocity(frontLeft.getSelectedSensorVelocity()), config.encoderTicksPer100msToVelocity(frontRight.getSelectedSensorVelocity()), config.encoderTicksPer100msToVelocity(backLeft.getSelectedSensorVelocity()), config.encoderTicksPer100msToVelocity(backRight.getSelectedSensorVelocity()))).vxMetersPerSecond);
    shuffleboardTab.addNumber("Chassis y Speed (Meters per Second)", () -> kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(config.encoderTicksPer100msToVelocity(frontLeft.getSelectedSensorVelocity()), config.encoderTicksPer100msToVelocity(frontRight.getSelectedSensorVelocity()), config.encoderTicksPer100msToVelocity(backLeft.getSelectedSensorVelocity()), config.encoderTicksPer100msToVelocity(backRight.getSelectedSensorVelocity()))).vxMetersPerSecond);
    shuffleboardTab.addNumber("Chassis omega Speed (Radians per Second)", () -> kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(config.encoderTicksPer100msToVelocity(frontLeft.getSelectedSensorVelocity()), config.encoderTicksPer100msToVelocity(frontRight.getSelectedSensorVelocity()), config.encoderTicksPer100msToVelocity(backLeft.getSelectedSensorVelocity()), config.encoderTicksPer100msToVelocity(backRight.getSelectedSensorVelocity()))).vxMetersPerSecond);
  }
}
