// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
import frc.team1891.common.led.LEDStrip.LEDPattern;
import frc.team1891.common.led.LEDStrip.LEDPatterns;

// Basic command that interrupts the LEDDefaultCommand and sets all LEDs to a given color
public class CustomColor extends CommandBase {
  private final LEDs leds;
  private final int r, g, b;
  public CustomColor(LEDs leds, int r, int g, int b) {
    addRequirements(leds);
    this.leds = leds;
    this.r = r;
    this.g = g;
    this.b = b;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.start();
    leds.setPattern(LEDPattern.setRGB(r, g, b));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setPattern(LEDPatterns.OFF);
    leds.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

