// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.drivetrains.motorcontrollers.TalonFXController;

/** Drivetrain base for a differential drivetrain. */
@SuppressWarnings("unused")
public abstract class DifferentialDrivetrain extends Drivetrain {
  protected final DifferentialDriveKinematics kinematics;
  protected final DifferentialDrivePoseEstimator poseEstimator;

  protected final DifferentialDrive differentialDrive;

  private final TalonFXController left, right;

  /**
   * Creates a new differential drivetrain.
   * @param config the drivetrain config
   * @param trackWidthMeters the width of the wheel base
   * @param gyro the gyro used by the drivetrain ({@link Gyro})
   * @param leftMaster the left drive motor
   * @param rightMaster the right drive motor
   */
  public DifferentialDrivetrain(
    DrivetrainConfig config,
    double trackWidthMeters,
    Gyro gyro,
    WPI_TalonFX leftMaster,
    WPI_TalonFX rightMaster
  ) {
    this(
      config,
      new DifferentialDriveKinematics(trackWidthMeters),
      gyro,
      leftMaster,
      rightMaster
    );
  }

  /**
   * Creates a new differential drivetrain.
   * @param config the drivetrain config
   * @param kinematics the kinematics for this drivetrain
   * @param gyro the gyro used by the drivetrain ({@link Gyro})
   * @param leftMaster the left drive motor
   * @param rightMaster the right drive motor
   */
  public DifferentialDrivetrain(
    DrivetrainConfig config,
    DifferentialDriveKinematics kinematics,
    Gyro gyro,
    WPI_TalonFX leftMaster,
    WPI_TalonFX rightMaster
  ) {
    this(config, kinematics, gyro, new TalonFXController(leftMaster, config), new TalonFXController(rightMaster, config));
  }

  /**
   * Creates a new differential drivetrain.
   * @param config the drivetrain config
   * @param trackWidthMeters the width of the wheel base
   * @param gyro the gyro used by the drivetrain ({@link Gyro})
   * @param leftMaster the left drive motor
   * @param rightMaster the right drive motor
   */
  public DifferentialDrivetrain(
    DrivetrainConfig config,
    double trackWidthMeters,
    Gyro gyro,
    TalonFXController leftMaster,
    TalonFXController rightMaster
  ) {
    this(
      config,
      new DifferentialDriveKinematics(trackWidthMeters),
      gyro,
      leftMaster,
      rightMaster
    );
  }

  /**
   * Creates a new differential drivetrain.
   * @param config the drivetrain config
   * @param kinematics the kinematics for this drivetrain
   * @param gyro the gyro used by the drivetrain ({@link Gyro})
   * @param leftMaster the left drive motor
   * @param rightMaster the right drive motor
   */
  public DifferentialDrivetrain(
    DrivetrainConfig config,
    DifferentialDriveKinematics kinematics,
    Gyro gyro,
    TalonFXController leftMaster,
    TalonFXController rightMaster
  ) {
    super(config, gyro);

    this.kinematics = kinematics;
    this.poseEstimator = new DifferentialDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            leftMaster.getPosition(),
            rightMaster.getPosition(),
            new Pose2d());

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
   * Drives the robot using {@link DifferentialDrive#tankDrive(double, double, boolean)}.
   * @param leftSpeed speed of left side
   * @param rightSpeed speed of right side
   * @param squaredInputs square joystick inputs for more controlled movements at low speed
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed, squaredInputs);
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

  /**
   * Drive the robot with direct control over the speed of each side of the drivetrain.
   * @param wheelSpeeds speed of wheels as {@link DifferentialDriveWheelSpeeds}
   */
  public void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    wheelSpeeds.desaturate(config.chassisMaxVelocityMetersPerSecond());
    left.setVelocity(wheelSpeeds.leftMetersPerSecond);
    right.setVelocity(wheelSpeeds.rightMetersPerSecond);
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
   * Returns the speeds of the left and right sides as a {@link DifferentialDriveWheelSpeeds}.
   * @return the speeds of the wheels
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left.getVelocity(), right.getVelocity());
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void stop() {
    differentialDrive.stopMotor();
  }

  @Override
  public void resetOdometry(Pose2d pose2d) {
    resetEncoders();
    poseEstimator.resetPosition(
            gyro.getRotation2d(),
            left.getPosition(),
            right.getPosition(),
            pose2d);
  }

  /**
   * Resets the encoders of the left and right motors
   */
  public void resetEncoders() {
    left.resetPosition();
    right.resetPosition();
  }

  @Override
  public void updateOdometry() {
    poseEstimator.update(
            gyro.getRotation2d(),
            config.encoderTicksToDistance(left.getPosition()),
            config.encoderTicksToDistance(right.getPosition()));
  }

  @Override
  protected void configureSmartDashboard() {
    super.configureSmartDashboard();
    LazyDashboard.addNumber("Drivetrain/leftPosition", left::getPosition);
    LazyDashboard.addNumber("Drivetrain/leftVelocity", left::getVelocity);
    LazyDashboard.addNumber("Drivetrain/rightPosition", right::getPosition);
    LazyDashboard.addNumber("Drivetrain/rightVelocity", right::getVelocity);
  }
}
