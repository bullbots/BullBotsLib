// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DrivetrainConstants.BackLeft;
import frc.robot.Constants.DrivetrainConstants.BackRight;
import frc.robot.Constants.DrivetrainConstants.FrontLeft;
import frc.robot.Constants.DrivetrainConstants.FrontRight;
import frc.robot.Robot;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.drivetrains.SwerveDrivetrain;
import frc.team1891.common.drivetrains.swervecontrollers.DriveController;
import frc.team1891.common.drivetrains.swervecontrollers.FalconDriveController;
import frc.team1891.common.drivetrains.swervecontrollers.FalconSteerController;
import frc.team1891.common.drivetrains.swervecontrollers.SwerveModule;
import frc.team1891.common.hardware.SimNavX;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Drivetrain extends SwerveDrivetrain {
  private static Drivetrain instance = null;
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  private static final SimNavX navX = new SimNavX();

  private static final WPI_TalonFX frontLeftDriveMotor = new WPI_TalonFX(FrontLeft.DRIVE_CHANNEL);
  private static final DriveController frontLeftDriveController = new FalconDriveController(frontLeftDriveMotor, CONFIG);
  private static final WPI_TalonFX frontLeftSteerMotor = new WPI_TalonFX(FrontLeft.STEER_CHANNEL);
  private static final WPI_CANCoder frontLeftEncoder = new WPI_CANCoder(FrontLeft.CANCODER_CHANNEL);
  private static final FalconSteerController frontLeftSteerController = new FalconSteerController(frontLeftSteerMotor, frontLeftEncoder, steerP, steerI, steerD);
  private static final SwerveModule frontLeft = new SwerveModule(frontLeftDriveController, frontLeftSteerController);
  private static final WPI_TalonFX frontRightDriveMotor = new WPI_TalonFX(FrontRight.DRIVE_CHANNEL);
  private static final DriveController frontRightDriveController = new FalconDriveController(frontRightDriveMotor, CONFIG);
  private static final WPI_TalonFX frontRightSteerMotor = new WPI_TalonFX(FrontRight.STEER_CHANNEL);
  private static final WPI_CANCoder frontRightEncoder = new WPI_CANCoder(FrontRight.CANCODER_CHANNEL);
  private static final FalconSteerController frontRightSteerController = new FalconSteerController(frontRightSteerMotor, frontRightEncoder, steerP, steerI, steerD);
  private static final SwerveModule frontRight = new SwerveModule(frontRightDriveController, frontRightSteerController);
  private static final WPI_TalonFX backLeftDriveMotor = new WPI_TalonFX(BackLeft.DRIVE_CHANNEL);
  private static final DriveController backLeftDriveController = new FalconDriveController(backLeftDriveMotor, CONFIG);
  private static final WPI_TalonFX backLeftSteerMotor = new WPI_TalonFX(BackLeft.STEER_CHANNEL);
  private static final WPI_CANCoder backLeftEncoder = new WPI_CANCoder(BackLeft.CANCODER_CHANNEL);
  private static final FalconSteerController backLeftSteerController = new FalconSteerController(backLeftSteerMotor, backLeftEncoder, steerP, steerI, steerD);
  private static final SwerveModule backLeft = new SwerveModule(backLeftDriveController, backLeftSteerController);
  private static final WPI_TalonFX backRightDriveMotor = new WPI_TalonFX(BackRight.DRIVE_CHANNEL);
  private static final DriveController backRightDriveController = new FalconDriveController(backRightDriveMotor, CONFIG);
  private static final WPI_TalonFX backRightSteerMotor = new WPI_TalonFX(BackRight.STEER_CHANNEL);
  private static final WPI_CANCoder backRightEncoder = new WPI_CANCoder(BackRight.CANCODER_CHANNEL);
  private static final FalconSteerController backRightSteerController = new FalconSteerController(backRightSteerMotor, backRightEncoder, steerP, steerI, steerD);
  private static final SwerveModule backRight = new SwerveModule(backRightDriveController, backRightSteerController);

  public Drivetrain() {
    super(
      CONFIG, 
      WHEEL_BASE_WIDTH_METERS,
      WHEEL_BASE_LENGTH_METERS,
      navX,
      frontLeft,
      frontRight,
      backLeft,
      backRight,
      MODULE_MAX_VELOCITY
    );
    
    // These values from SwerveDrivetrain don't update properly in the simulator normally, but they're helpful to see.
    if (Robot.isSimulation()) {
      LazyDashboard.addNumber("Drivetrain/xSpeed (Meters per Second)", 10, () -> simSpeeds.vxMetersPerSecond);
      LazyDashboard.addNumber("Drivetrain/ySpeed (Meters per Second)", 10, () -> simSpeeds.vyMetersPerSecond);
      LazyDashboard.addNumber("Drivetrain/omegaSpeed (Radians per Second)", 10, () -> simSpeeds.omegaRadiansPerSecond);
    }

    configureSmartDashboard();
  }

  // Calcuate and updated the robot's position in the simulator
  private ChassisSpeeds simSpeeds = new ChassisSpeeds();
  @Override
  public void simulationPeriodic() {
    // Calculate the theoretical chassis speed based on the desired position of the modules
    simSpeeds = kinematics.toChassisSpeeds(
      frontLeft.getDesiredSwerveModuleState(),
      frontRight.getDesiredSwerveModuleState(),
      backLeft.getDesiredSwerveModuleState(),
      backRight.getDesiredSwerveModuleState()
    );

    // update pose estimation and gyro readings according to theoretical chassis speeds
    navX.setRadians(navX.getRadians() - simSpeeds.omegaRadiansPerSecond * .02);
    Pose2d newPose = poseEstimator.getEstimatedPosition().plus(
      new Transform2d(
        new Translation2d(
          simSpeeds.vxMetersPerSecond * .02,
          simSpeeds.vyMetersPerSecond * .02
        ),
        new Rotation2d()
      )
    );

    poseEstimator.resetPosition(navX.getRotation2d(), getSwerveModulePositions(), newPose);
  }

  // Don't override periodic without calling super.periodic() because that is how odometry is updated, and updates certain SmartDashboard values
  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}
