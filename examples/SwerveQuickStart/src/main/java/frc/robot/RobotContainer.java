// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.Drivetrain;
import frc.team1891.common.trajectory.HolonomicTrajectoryCommandGenerator;
import static frc.robot.Constants.DrivetrainConstants.*;

public class RobotContainer {
  // Subsystems
  private final Drivetrain drivetrain = Drivetrain.getInstance();

  // Controllers
  private final XboxController controller = new XboxController(0);

  // Triggers
  private final Trigger robotCentricDrive = new JoystickButton(controller, XboxController.Button.kA.value);
  private final Trigger lockWheels = new JoystickButton(controller, XboxController.Button.kX.value);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
      new JoystickDrive(
        drivetrain,
        () -> controller.getLeftY(),
        () -> controller.getLeftX(),
        () -> controller.getRightX(),
        () -> !robotCentricDrive.getAsBoolean()
      )
    );

    lockWheels.whileTrue(new RunCommand(() -> {
      drivetrain.moduleXConfiguration();
    }, drivetrain));
  }

  public Command getAutonomousCommand() {
    HolonomicTrajectoryCommandGenerator.setTranslationalPID(translationalP, translationalI, translationalD);
    HolonomicTrajectoryCommandGenerator.setRotationalPID(rotationalP, rotationalI, rotationalD);

    return HolonomicTrajectoryCommandGenerator.generate(drivetrain, true,
      new Pair<Pose2d, Rotation2d>(new Pose2d(), new Rotation2d()),
      new Pair<Pose2d, Rotation2d>(new Pose2d(4,2, Rotation2d.fromDegrees(80)), Rotation2d.fromDegrees(180)),
      new Pair<Pose2d, Rotation2d>(new Pose2d(8,4, Rotation2d.fromDegrees(40)), new Rotation2d())
    );
  }
}
