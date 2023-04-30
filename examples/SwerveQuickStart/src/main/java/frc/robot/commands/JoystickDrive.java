// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final DoubleSupplier forward, strafe, twist;
  private final BooleanSupplier fieldOrientedDrive;
  /**
   * Creates a new command that drives the robot according to joystick inputs (Double Suppliers) and determines field or robot oriented drive based on the boolean supplier.
   */
  public JoystickDrive(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier twist, BooleanSupplier fieldOrientedDrive) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.strafe = strafe;
    this.twist = twist;
    this.fieldOrientedDrive = fieldOrientedDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.holonomicDrive(-forward.getAsDouble(), -strafe.getAsDouble(), -twist.getAsDouble(), fieldOrientedDrive.getAsBoolean()); // negative is forward on the joystick; chassis left is positive while joystick right is positive.
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
