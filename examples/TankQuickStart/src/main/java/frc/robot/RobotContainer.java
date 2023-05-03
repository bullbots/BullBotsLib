// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  // Subsystems
  private final Drivetrain drivetrain = Drivetrain.getInstance();

  // Controllers
  private final XboxController controller = new XboxController(0);

  // Triggers
  private final Trigger resetOdometry = new JoystickButton(controller, XboxController.Button.kA.value);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
      new JoystickDrive(
        drivetrain,
        () -> controller.getLeftY(),
        () -> controller.getRightX()
      )
    );

    resetOdometry.onTrue(new InstantCommand(() -> {
      drivetrain.resetGyro();
      drivetrain.resetOdometry();
      drivetrain.resetEncoders();
    }));
  }

  public Command getAutonomousCommand() {
    // See https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/creating-following-trajectory.html for more information.

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.CHASSIS_MAX_VELOCITY,
                Constants.CHASSIS_MAX_ACCELERATION)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(drivetrain.getKinematics());

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
              new Translation2d(1, 1.75),
              new Translation2d(2, .75)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            drivetrain::getPose2d,
            new RamseteController(1, .5),
            new SimpleMotorFeedforward(
                1,
                0,
                0),
            drivetrain.getKinematics(),
            drivetrain::getWheelSpeeds,
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            // RamseteCommand passes volts to the callback
            (leftVolts, rightVolts) -> {
              drivetrain.tankDrive(leftVolts / RobotController.getBatteryVoltage(), rightVolts / RobotController.getBatteryVoltage(), false);
            },
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.stop());
  }
}
