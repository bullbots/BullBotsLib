// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.drivetrains.swervemodules.SwerveModule;

import java.security.InvalidParameterException;

/** Drivetrain base for a swerve drivetrain. */
@SuppressWarnings("unused")
public abstract class SwerveDrivetrain extends HolonomicDrivetrain {
  protected final SwerveDrivePoseEstimator poseEstimator;
//  private final Translation2d frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation;
  private final Translation2d[] moduleLocations;
  protected final SwerveDriveKinematics kinematics;

//  private final SwerveModule frontLeft, frontRight, backLeft, backRight;
  private final SwerveModule[] modules;

  private final double moduleMaxVelocityMetersPerSecond;

  private final Field2d modulesField;
  private final double modulesFieldXOffset, modulesFieldYOffset;

  /**
   * Creates a new SwerveDrivetrain, assuming the center of the robot is the center of a 4 wheel drivebase.
   * @param config the config of the drivetrain
   * @param gyro the gyro of the drivetrain
   * @param moduleMaxVelocityMetersPerSecond the maximum velocity of an individual module
   * @param driveBaseWidth the width between left and right modules
   * @param driveBaseLength the length between front and back modules
   * @param frontLeft the front left module
   * @param frontRight the front right module
   * @param backLeft the back left module
   * @param backRight the back right module
   */
  public SwerveDrivetrain(
    DrivetrainConfig config,
    Gyro gyro,
    double moduleMaxVelocityMetersPerSecond,
    double driveBaseWidth,
    double driveBaseLength,
    SwerveModule frontLeft,
    SwerveModule frontRight,
    SwerveModule backLeft,
    SwerveModule backRight
  ) {
    this(
      config,
      gyro,
      moduleMaxVelocityMetersPerSecond,
      new Translation2d(driveBaseLength / 2d, driveBaseWidth / 2d),
      new Translation2d(driveBaseLength / 2d, -driveBaseWidth / 2d),
      new Translation2d(-driveBaseLength / 2d, driveBaseWidth / 2d),
      new Translation2d(-driveBaseLength / 2d, -driveBaseWidth / 2d),
      frontLeft,
      frontRight,
      backLeft,
      backRight
    );
  }

  /**
   * Creates a new SwerveDrivetrain with a 4 wheel drivebase.
   * @param config the config of the drivetrain
   * @param gyro the gyro of the drivetrain
   * @param moduleMaxVelocityMetersPerSecond the maximum velocity of an individual module
   * @param frontLeftLocation the location of the front left module relative to the robot center
   * @param frontRightLocation the location of the front right module relative to the robot center
   * @param backLeftLocation the location of the back left module relative to the robot center
   * @param backRightLocation the location of the back right module relative to the robot center
   * @param frontLeft the front left module
   * @param frontRight the front right module
   * @param backLeft the back left module
   * @param backRight the back right module
   */
  public SwerveDrivetrain(
    DrivetrainConfig config,
    Gyro gyro,
    double moduleMaxVelocityMetersPerSecond,
    Translation2d frontLeftLocation,
    Translation2d frontRightLocation,
    Translation2d backLeftLocation,
    Translation2d backRightLocation,
    SwerveModule frontLeft,
    SwerveModule frontRight,
    SwerveModule backLeft,
    SwerveModule backRight
  ) {
    super(config, gyro);

    this.moduleMaxVelocityMetersPerSecond = moduleMaxVelocityMetersPerSecond;

    modules = new SwerveModule[4];
    modules[0] = frontLeft;
    modules[1] = frontRight;
    modules[2] = backLeft;
    modules[3] = backRight;

    moduleLocations = new Translation2d[4];
    moduleLocations[0] = frontLeftLocation;
    moduleLocations[1] = frontRightLocation;
    moduleLocations[2] = backLeftLocation;
    moduleLocations[3] = backRightLocation;

    this.kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

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

  /**
   * Creates a new SwerveDrivetrain with a custom number of wheels.
   * @param config the config of the drivetrain
   * @param gyro the gyro of the drivetrain
   * @param moduleMaxVelocityMetersPerSecond the maximum velocity of an individual module
   * @param modulePairs a pair for each module in the drivetrain, giving th actual module along with its position
   *                    relative to the robot's center
   */
  @SafeVarargs
  public SwerveDrivetrain(
          DrivetrainConfig config,
          Gyro gyro,
          double moduleMaxVelocityMetersPerSecond,
          Pair<Translation2d, SwerveModule> ... modulePairs
  ) {
    super(config, gyro);

    moduleLocations = new Translation2d[modulePairs.length];
    modules = new SwerveModule[modulePairs.length];
    for (int i = 0; i < modulePairs.length; i++) {
      moduleLocations[i] = modulePairs[i].getFirst();
      modules[i] = modulePairs[i].getSecond();
    }

    this.moduleMaxVelocityMetersPerSecond = moduleMaxVelocityMetersPerSecond;

    this.kinematics = new SwerveDriveKinematics(moduleLocations);

//    this.odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getSwerveModulePositions());
    // TODO: new Pose2d() assumes the robot starts at 0,0, with isn't a safe assumption.
    // Potential 'solution' is just expecting them to call resetOdometry() with their starting pose.
    this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), getSwerveModulePositions(), new Pose2d());



    double forwardMost = moduleLocations[0].getX();
    double backwardMost = moduleLocations[0].getX();
    double leftwardMost = moduleLocations[0].getY();
    double rightwardMost = moduleLocations[0].getY();
    for (Translation2d moduleLocation : moduleLocations) {
      if (moduleLocation.getX() > forwardMost) {
        forwardMost = moduleLocation.getX();
      }
      if (moduleLocation.getX() < backwardMost) {
        backwardMost = moduleLocation.getX();
      }
      if (moduleLocation.getY() > leftwardMost) {
        leftwardMost = moduleLocation.getY();
      }
      if (moduleLocation.getY() < rightwardMost) {
        rightwardMost = moduleLocation.getY();
      }
    }

    this.modulesField = new Field2d();
    this.modulesFieldXOffset = Math.abs(forwardMost) + Math.abs(backwardMost);
    this.modulesFieldYOffset = Math.abs(leftwardMost) + Math.abs(rightwardMost);
    modulesField.setRobotPose(modulesFieldXOffset, modulesFieldYOffset, new Rotation2d());
    SmartDashboard.putBoolean("Modules Robot Relative", true);
    SmartDashboard.putBoolean("Modules Show Desired States", true);
    SmartDashboard.putData("Modules (Field2d)", modulesField);
  }

  @Override
  public void holonomicDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= config.chassisMaxVelocityMetersPerSecond();
    ySpeed *= config.chassisMaxVelocityMetersPerSecond();
    rot *= config.chassisMaxAngularVelocityRadiansPerSecond();

    fromChassisSpeeds(
      fieldRelative?
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
      :
        new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
  }

  @Override
  public void fromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // Check if chassis is stationary, if so, don't move the modules
    if (Math.abs(chassisSpeeds.vxMetersPerSecond) == 0 && Math.abs(chassisSpeeds.vyMetersPerSecond) == 0 && Math.abs(chassisSpeeds.omegaRadiansPerSecond) == 0) {
      stop();
    } else {
      // Keep velocity below the max speed
      if (chassisSpeeds.vxMetersPerSecond* chassisSpeeds.vxMetersPerSecond+ chassisSpeeds.vyMetersPerSecond* chassisSpeeds.vyMetersPerSecond > config.chassisMaxVelocityMetersPerSecond()) {
        double theta = Math.atan2(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond);
        chassisSpeeds.vxMetersPerSecond = Math.cos(theta) * config.chassisMaxVelocityMetersPerSecond();
        chassisSpeeds.vyMetersPerSecond = Math.sin(theta) * config.chassisMaxVelocityMetersPerSecond();
      }
      setSwerveModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
    }
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  /**
   * Sets the modules to an x shape to avoid moving unintentionally.
   */
  public void moduleXConfiguration() {
    setSwerveModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    });
  }

  /**
   * Sets the 4 swerve modules to the desired states.
   * @param swerveModuleStates An array of length 4, of the desired module states
   */
  public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
    if (swerveModuleStates.length != modules.length) {
      throw new InvalidParameterException(
              "Number of desired swerve module states does not match number of modules in drivetrain.");
    }

    // Normalize wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, moduleMaxVelocityMetersPerSecond);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredSwerveModuleState(swerveModuleStates[i]);
    }
  }

  /**
   * Returns the positions of the modules as a {@link SwerveModulePosition} array.
   * @return the positions of the modules
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    final SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getSwerveModulePosition();
    }
    return positions;
  }

  /**
   * Returns the states of the modules as a {@link SwerveModuleState} array.
   * @return the states of the modules
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    final SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getSwerveModuleState();
    }
    return states;
  }

  /**
   * Returns the {@link SwerveDriveKinematics} of this drivetrain.
   * @return kinematics
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Stops all motors.
   */
  public void stop() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  
  @Override
  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void resetOdometry(Pose2d pose2d) {
      poseEstimator.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), pose2d);
  }

  @Override
  public void updateOdometry() {
    // TODO: Add timestamp to estimate
    poseEstimator.update(
      gyro.getRotation2d(),
      getSwerveModulePositions()
    );
  }

  @Override
  protected void configureSmartDashboard() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].configureSmartDashboard("Module " + i);
    }
    LazyDashboard.addNumber("Drivetrain/gyroRadians", gyro::getAngle);
    LazyDashboard.addNumber("Drivetrain/xSpeed (Meters per Second)", () -> getChassisSpeeds().vxMetersPerSecond);
    LazyDashboard.addNumber("Drivetrain/ySpeed (Meters per Second)", () -> getChassisSpeeds().vyMetersPerSecond);
    LazyDashboard.addNumber("Drivetrain/omegaSpeed (Radians per Second)", () -> getChassisSpeeds().omegaRadiansPerSecond);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (smartDashboardEnabled) {
      final boolean modulesRobotRelative = SmartDashboard.getBoolean("Modules Robot Relative", true);
      final boolean modulesShowDesiredStates = SmartDashboard.getBoolean("Modules Show Desired States", true);
      if (modulesRobotRelative) {
        modulesField.setRobotPose(modulesFieldXOffset, modulesFieldYOffset, getPose2d().getRotation());
      } else {
        modulesField.setRobotPose(modulesFieldXOffset, modulesFieldYOffset, new Rotation2d());
      }
      for (int i = 0; i < modules.length; i++) {
        Translation2d moduleTranslation = moduleLocations[i];
        if (modulesRobotRelative) {
          moduleTranslation = moduleTranslation.rotateBy(getPose2d().getRotation());
        }
        modulesField.getObject("module"+i).setPose(moduleTranslation.getX() + modulesFieldXOffset, moduleTranslation.getY() + modulesFieldYOffset, modules[i].getAngleRotation2d().rotateBy(modulesRobotRelative ? getPose2d().getRotation() : new Rotation2d()).rotateBy(modules[i].getSwerveModuleState().speedMetersPerSecond < 0 ? Rotation2d.fromRotations(.5) : new Rotation2d()));
        if (modulesShowDesiredStates) {
          modulesField.getObject("module"+i+"Desired").setPose(moduleTranslation.getX() + modulesFieldXOffset, moduleTranslation.getY() + modulesFieldYOffset, modules[i].getAngleRotation2d().rotateBy(modulesRobotRelative ? getPose2d().getRotation() : new Rotation2d()).rotateBy(modules[i].getDesiredSwerveModuleState().speedMetersPerSecond < 0 ? Rotation2d.fromRotations(.5) : new Rotation2d()));
        }
      }
    }
  }
}
