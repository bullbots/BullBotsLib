// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;
import frc.team1891.common.drivetrains.DifferentialDrivetrain;
import frc.team1891.common.hardware.SimNavX;

public class Drivetrain extends DifferentialDrivetrain {
  private static Drivetrain instance = null;
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }
  private static final SimNavX navX = new SimNavX();

  private static final WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.FRONT_LEFT_CAN_ID);
  private static final WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.FRONT_RIGHT_CAN_ID);
  private static final WPI_TalonFX leftSlave = new WPI_TalonFX(Constants.BACK_LEFT_CAN_ID);
  private static final WPI_TalonFX rightSlave = new WPI_TalonFX(Constants.BACK_RIGHT_CAN_ID);
  
  private Drivetrain() {
    super(Constants.CONFIG, Constants.TRACK_WIDTH_METERS, navX, leftMaster, rightMaster);

    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightMaster.setInverted(true);
    rightSlave.setInverted(InvertType.FollowMaster);

    configureSmartDashboard();
  }
  
  // Don't override periodic without calling super.periodic() because that is how odometry is updated
  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
  
  // Calcuate and updated the robot's position in the simulator
  private ChassisSpeeds simSpeeds = new ChassisSpeeds();
  @Override
  public void simulationPeriodic() {
    // Calculate the theoretical chassis speed based on the desired position of the modules
    simSpeeds = kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(
        leftMaster.get()*config.chassisMaxVelocityMetersPerSecond(),
        rightMaster.get()*config.chassisMaxVelocityMetersPerSecond())
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

    poseEstimator.resetPosition(
      navX.getRotation2d(),
      config.encoderTicksToDistance(leftMaster.getSelectedSensorPosition()),
      config.encoderTicksToDistance(rightMaster.getSelectedSensorPosition()),
      newPose);
  }
}
