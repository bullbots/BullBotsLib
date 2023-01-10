// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.*;
import frc.team1891.common.hardware.NavX;

/** Swerve Drivetrain base. */
public class SwerveDrivetrain extends HolonomicDrivetrain {
  public static final double MAX_VOLTAGE = 12.0;
  protected static double DEADBAND = .15;

  protected final SwerveDriveOdometry odometry;
  protected final SwerveDriveKinematics kinematics;

  public static final SwerveModuleState[] EMPTY_SWERVE_MODULE_STATES = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()
  };


  private final SwerveModule frontLeft, frontRight, backLeft, backRight;

  /** Creates a new Drivetrain. */
  public SwerveDrivetrain(
    DrivetrainConfig config,
    SwerveDriveKinematics kinematics,
    SwerveDriveOdometry odometry,
    NavX gyro,
    SwerveModule frontLeft,
    SwerveModule frontRight,
    SwerveModule backLeft,
    SwerveModule backRight
  ) {
    super(config, gyro);

    this.odometry = odometry;
    this.kinematics = kinematics;

    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;
  }

  public void holonomicDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= config.chassisMaxVelocityMetersPerSecond;
    ySpeed *= config.chassisMaxVelocityMetersPerSecond;
    rot *= config.chassisMaxAngularVelocityRadiansPerSecond;

    SwerveModuleState[] swerveModuleStates;
    if (Math.abs(xSpeed) < DEADBAND && Math.abs(ySpeed) < DEADBAND && Math.abs(rot) < DEADBAND) {
      swerveModuleStates = new SwerveModuleState[] {
        new SwerveModuleState(0, frontLeft.getState().angle),
        new SwerveModuleState(0, frontRight.getState().angle),
        new SwerveModuleState(0, backLeft.getState().angle),
        new SwerveModuleState(0, backRight.getState().angle)
      };
    } else {
      swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative?
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
        :
            new ChassisSpeeds(xSpeed, ySpeed, rot)
        );
    }

    setSwerveModuleStates(swerveModuleStates);
  }

  @Override
  public void fromChassisSpeeds(ChassisSpeeds speeds) {
      setSwerveModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  /**
   * Sets the 4 swerve modules to the desired states.
   * @param swerveModuleStates An array of length 4, of the desired module states
   */
  public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
    // Normalize wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, config.chassisMaxVelocityMetersPerSecond);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Stops all drive motors.
   */
  public void stop() {
    frontLeft.setDesiredState(0, frontLeft.getState().angle);
    frontRight.setDesiredState(0, frontRight.getState().angle);
    backLeft.setDesiredState(0, backLeft.getState().angle);
    backRight.setDesiredState(0, backRight.getState().angle);
  }

  
  @Override
  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  @Override
  public void resetOdometry(Pose2d pose2d) {
      odometry.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), pose2d);
  }

  /**
   * Updates the field relative position of the robot.
   * 
   * Odometry is measured in meters.
   */
  public void updateOdometry() {
    odometry.update(
      gyro.getRotation2d(),
      getSwerveModulePositions()
    );
  }
}
