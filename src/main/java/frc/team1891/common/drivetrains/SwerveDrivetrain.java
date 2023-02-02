// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.drivetrains.swervemodules.SwerveModule;
import frc.team1891.common.hardware.NavX;

/** Swerve Drivetrain base. */
public class SwerveDrivetrain extends HolonomicDrivetrain {
  public static final SwerveModuleState[] EMPTY_SWERVE_MODULE_STATES = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()
  };

  protected final SwerveDrivePoseEstimator poseEstimator;
  private final Translation2d frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation;
  protected final SwerveDriveKinematics kinematics;

  private final SwerveModule frontLeft, frontRight, backLeft, backRight;

  private final Field2d modulesField;
  private final double modulesFieldXOffset, modulesFieldYOffset;

  /**
   * Creates a new SwerveDrivetrain, assuming the center of the robot is the center of the drivebase.
   * @param config the config of the drivetrain
   * @param driveBaseWidth the width between left and right modules
   * @param driveBaseLength the length between front and back modules
   * @param gyro the gyro of the drivetrain
   * @param frontLeft the front left module
   * @param frontRight the front right module
   * @param backLeft the back left module
   * @param backRight the back right module
   */
  public SwerveDrivetrain(
    DrivetrainConfig config,
    double driveBaseWidth,
    double driveBaseLength,
    NavX gyro,
    SwerveModule frontLeft,
    SwerveModule frontRight,
    SwerveModule backLeft,
    SwerveModule backRight
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
   * Creates a new SwerveDrivetrain.
   * @param config the config of the drivetrain
   * @param frontLeftLocation the location of the front left module relative to the robot center
   * @param frontRightLocation the location of the front right module relative to the robot center
   * @param backLeftLocation the location of the back left module relative to the robot center
   * @param backRightLocation the location of the back right module relative to the robot center
   * @param gyro the gyro of the drivetrain
   * @param frontLeft the front left module
   * @param frontRight the front right module
   * @param backLeft the back left module
   * @param backRight the back right module
   */
  public SwerveDrivetrain(
    DrivetrainConfig config,
    Translation2d frontLeftLocation,
    Translation2d frontRightLocation,
    Translation2d backLeftLocation,
    Translation2d backRightLocation,
    NavX gyro,
    SwerveModule frontLeft,
    SwerveModule frontRight,
    SwerveModule backLeft,
    SwerveModule backRight
  ) {
    super(config, gyro);

    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;

    this.frontLeftLocation = frontLeftLocation;
    this.frontRightLocation = frontRightLocation;
    this.backLeftLocation = backLeftLocation;
    this.backRightLocation = backRightLocation;

    this.kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

//    this.odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getSwerveModulePositions());
    // TODO: new Pose2d() assumes the robot starts at 0,0, with isn't a safe assumption.
      // Potential 'solution' is just expecting them to call resetOdometry() with their starting pose.
    this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), getSwerveModulePositions(), new Pose2d());

    this.modulesField = new Field2d();
    this.modulesFieldXOffset = Math.abs(frontLeftLocation.getX()) + Math.abs(backLeftLocation.getX());
    this.modulesFieldYOffset = Math.abs(frontLeftLocation.getY()) + Math.abs(backLeftLocation.getY());
    modulesField.setRobotPose(modulesFieldXOffset, modulesFieldYOffset, new Rotation2d());
    SmartDashboard.putBoolean("Modules Robot Relative", true);
    SmartDashboard.putBoolean("Modules Show Desired States", true);
    SmartDashboard.putData("Modules (Field2d)", modulesField);
  }

  public void holonomicDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= config.chassisMaxVelocityMetersPerSecond;
    ySpeed *= config.chassisMaxVelocityMetersPerSecond;
    rot *= config.chassisMaxAngularVelocityRadiansPerSecond;

    fromChassisSpeeds(
      fieldRelative?
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
      :
        new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
  }

  @Override
  public void fromChassisSpeeds(ChassisSpeeds speeds) {
    // Check if chassis is stationary, if so, don't move the modules
    if (Math.abs(speeds.vxMetersPerSecond) == 0 && Math.abs(speeds.vyMetersPerSecond) == 0 && Math.abs(speeds.omegaRadiansPerSecond) == 0) {
      stop();
    } else {
      setSwerveModuleStates(kinematics.toSwerveModuleStates(speeds));
    }
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  /**
   * Sets the 4 swerve modules to the desired states.
   * @param swerveModuleStates An array of length 4, of the desired module states
   */
  public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
    // Normalize wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, config.chassisMaxVelocityMetersPerSecond);

    frontLeft.setDesiredSwerveModuleState(swerveModuleStates[0]);
    frontRight.setDesiredSwerveModuleState(swerveModuleStates[1]);
    backLeft.setDesiredSwerveModuleState(swerveModuleStates[2]);
    backRight.setDesiredSwerveModuleState(swerveModuleStates[3]);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getSwerveModulePosition(),
      frontRight.getSwerveModulePosition(),
      backLeft.getSwerveModulePosition(),
      backRight.getSwerveModulePosition()
    };
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getSwerveModuleState(),
      frontRight.getSwerveModuleState(),
      backLeft.getSwerveModuleState(),
      backRight.getSwerveModuleState()
    };
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Stops all motors.
   */
  public void stop() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  
  @Override
  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void resetOdometry(Pose2d pose2d) {
      poseEstimator.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), pose2d);
  }

  /**
   * Updates the field relative position of the robot.
   * 
   * Odometry is measured in meters.
   */
  public void updateOdometry() {
    // TODO: Add timestamp to estimate
    poseEstimator.update(
      gyro.getRotation2d(),
      getSwerveModulePositions()
    );
  }

  @Override
  protected void configureSmartDashboard() {
    frontLeft.configureSmartDashboard("Front Left");
    frontRight.configureSmartDashboard("Front Right");
    backLeft.configureSmartDashboard("Back Left");
    backRight.configureSmartDashboard("Back Right");
    LazyDashboard.addNumber("Drivetrain/gyroRadians", gyro::getRadians);
    LazyDashboard.addNumber("Drivetrain/gyroDegrees", gyro::getDegrees);
    LazyDashboard.addNumber("Drivetrain/gyroDegreesLooped", gyro::getDegreesLooped);
    LazyDashboard.addNumber("Drivetrain/xSpeed (Meters per Second)", () -> getChassisSpeeds().vxMetersPerSecond);
    LazyDashboard.addNumber("Drivetrain/ySpeed (Meters per Second)", () -> getChassisSpeeds().vyMetersPerSecond);
    LazyDashboard.addNumber("Drivetrain/omegaSpeed (Radians per Second)", () -> getChassisSpeeds().omegaRadiansPerSecond);
  }

  @Override
  public void periodic() {
    super.periodic();

    // modulesField
    Translation2d frontLeftT = new Translation2d(frontLeftLocation.getX(), frontLeftLocation.getY());
    Translation2d frontRightT =  new Translation2d(frontRightLocation.getX(), frontRightLocation.getY());
    Translation2d backLeftT =  new Translation2d(backLeftLocation.getX(), backLeftLocation.getY());
    Translation2d backRightT =  new Translation2d(backRightLocation.getX(), backRightLocation.getY());
    if (SmartDashboard.getBoolean("Modules Robot Relative", true)) {
      frontLeftT = frontLeftT.rotateBy(gyro.getRotation2d());
      frontRightT = frontRightT.rotateBy(gyro.getRotation2d());
      backLeftT = backLeftT.rotateBy(gyro.getRotation2d());
      backRightT = backRightT.rotateBy(gyro.getRotation2d());
      modulesField.setRobotPose(modulesFieldXOffset, modulesFieldYOffset, gyro.getRotation2d());
    } else {
      modulesField.setRobotPose(modulesFieldXOffset, modulesFieldYOffset, new Rotation2d());
    }

    modulesField.getObject("frontLeft").setPose(frontLeftT.getX()+modulesFieldXOffset, frontLeftT.getY()+modulesFieldYOffset, frontLeft.getAngleRotation2d().rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()).rotateBy(frontLeft.getSwerveModuleState().speedMetersPerSecond < 0 ? Rotation2d.fromRotations(.5) : new Rotation2d()));
    modulesField.getObject("frontRight").setPose(frontRightT.getX()+modulesFieldXOffset, frontRightT.getY()+modulesFieldYOffset, frontRight.getAngleRotation2d().rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()).rotateBy(frontRight.getSwerveModuleState().speedMetersPerSecond < 0 ? Rotation2d.fromRotations(.5) : new Rotation2d()));
    modulesField.getObject("backLeft").setPose(backLeftT.getX()+modulesFieldXOffset, backLeftT.getY()+modulesFieldYOffset, backLeft.getAngleRotation2d().rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()).rotateBy(backLeft.getSwerveModuleState().speedMetersPerSecond < 0 ? Rotation2d.fromRotations(.5) : new Rotation2d()));
    modulesField.getObject("backRight").setPose(backRightT.getX()+modulesFieldXOffset, backRightT.getY()+modulesFieldYOffset, backRight.getAngleRotation2d().rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()).rotateBy(backRight.getSwerveModuleState().speedMetersPerSecond < 0 ? Rotation2d.fromRotations(.5) : new Rotation2d()));

    if (SmartDashboard.getBoolean("Modules Show Desired States", true)) {
      modulesField.getObject("frontLeftDesired").setPose(frontLeftT.getX() + modulesFieldXOffset, frontLeftT.getY() + modulesFieldYOffset, frontLeft.getDesiredSwerveModuleState().angle.rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));
      modulesField.getObject("frontRightDesired").setPose(frontRightT.getX() + modulesFieldXOffset, frontRightT.getY() + modulesFieldYOffset, frontRight.getDesiredSwerveModuleState().angle.rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));
      modulesField.getObject("backLeftDesired").setPose(backLeftT.getX() + modulesFieldXOffset, backLeftT.getY() + modulesFieldYOffset, backLeft.getDesiredSwerveModuleState().angle.rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));
      modulesField.getObject("backRightDesired").setPose(backRightT.getX() + modulesFieldXOffset, backRightT.getY() + modulesFieldYOffset, backRight.getDesiredSwerveModuleState().angle.rotateBy(SmartDashboard.getBoolean("Modules Robot Relative", true) ? gyro.getRotation2d() : new Rotation2d()));
    }
  }
}
