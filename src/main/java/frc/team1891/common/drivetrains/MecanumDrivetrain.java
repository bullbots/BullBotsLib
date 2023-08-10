// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.drivetrains.motorcontrollers.MotorController;
import frc.team1891.common.drivetrains.motorcontrollers.TalonFXController;

/** Drivetrain base for a mecanum drivetrain. */
@SuppressWarnings("unused")
public abstract class MecanumDrivetrain extends HolonomicDrivetrain {
  protected final MecanumDrivePoseEstimator poseEstimator;
  protected final MecanumDriveKinematics kinematics;

  private final MotorController frontLeft, frontRight, backLeft, backRight;

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
    Gyro gyro,
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
    Gyro gyro,
    WPI_TalonFX frontLeft,
    WPI_TalonFX frontRight,
    WPI_TalonFX backLeft,
    WPI_TalonFX backRight
  ) {
    this(
      config,
      frontLeftLocation,
      frontRightLocation,
      backLeftLocation,
      backRightLocation,
      gyro,
      new TalonFXController(frontLeft, config),
      new TalonFXController(frontRight, config),
      new TalonFXController(backLeft, config),
      new TalonFXController(backRight, config));
  }

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
    Gyro gyro,
    MotorController frontLeft,
    MotorController frontRight,
    MotorController backLeft,
    MotorController backRight
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
    Gyro gyro,
    MotorController frontLeft,
    MotorController frontRight,
    MotorController backLeft,
    MotorController backRight
  ) {
    super(config, gyro);

    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;

    this.kinematics = new MecanumDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation);

    this.poseEstimator = new MecanumDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            getWheelPositions(),
            new Pose2d());
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

    frontLeft.setVelocity(wheelSpeeds.frontLeftMetersPerSecond);
    frontRight.setVelocity(wheelSpeeds.frontRightMetersPerSecond);
    backLeft.setVelocity(wheelSpeeds.rearLeftMetersPerSecond);
    backRight.setVelocity(wheelSpeeds.rearRightMetersPerSecond);
  }

  /**
   * Returns the speeds of the left and right sides as a {@link MecanumDriveWheelSpeeds}.
   * @return the speeds of the wheels
   */
  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      frontLeft.getVelocity(),
      frontRight.getVelocity(),
      backLeft.getVelocity(),
      backRight.getVelocity()
    );
  }

  /**
   * Returns the positions of the left and right sides as a {@link MecanumDriveWheelPositions}.
   * @return the positions of the wheels
   */
  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
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
    frontLeft.stopMotor();
    frontRight.stopMotor();
    backLeft.stopMotor();
    backRight.stopMotor();
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
    LazyDashboard.addNumber("Drivetrain/frontLeftPosition", frontLeft::getPosition);
    LazyDashboard.addNumber("Drivetrain/frontLeftVelocity", frontLeft::getVelocity);
    LazyDashboard.addNumber("Drivetrain/frontRightPosition", frontRight::getPosition);
    LazyDashboard.addNumber("Drivetrain/frontRightVelocity", frontRight::getVelocity);
    LazyDashboard.addNumber("Drivetrain/backLeftPosition", backLeft::getPosition);
    LazyDashboard.addNumber("Drivetrain/backLeftVelocity", backLeft::getVelocity);
    LazyDashboard.addNumber("Drivetrain/backRightPosition", backRight::getPosition);
    LazyDashboard.addNumber("Drivetrain/backRightVelocity", backRight::getVelocity);
  }
}
