// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final DoubleSupplier forward, twist;

  /**
   * Creates a new command that drives the robot according to joystick inputs (Double Suppliers).
   */
  public JoystickDrive(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier twist) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.twist = twist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(-forward.getAsDouble(), -twist.getAsDouble()*.5, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
