// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.Segment;
import frc.team1891.common.led.LEDStripInterface;
import frc.team1891.common.led.LEDStrip.LEDPattern;
import frc.team1891.common.led.LEDStrip.LEDPatterns;

public class LEDDefaultCommand extends CommandBase {
  private final LEDs leds;
  public LEDDefaultCommand(LEDs leds) {
    addRequirements(leds);
    this.leds = leds;
  }

  public static boolean isDrivingWithAbsoluteAngle = false;
  public static boolean clawHasEStopped = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.start();
    leds.setPattern(Segment.MAIN, MAIN_PATTERN);
    leds.setPattern(Segment.STATUS, STATUS_PATTERN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setPattern(LEDPatterns.OFF);;
    leds.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Keeps the command running whenever the robot is on (outside test mode).
  @Override
  public boolean runsWhenDisabled() {
      return true;
  }

  private static final LEDPattern GOOD = new LEDPattern() {
    private int i = 0;
    @Override
    public void draw(LEDStripInterface leds) {
      i -= 2;
      if (i < 0) {
        i = 255;
      }
      leds.setAllRGB(0, i, 0);
    }
  };

  private static final Supplier<LEDPattern> STATUS_PATTERN = () -> {
    if (!DriverStation.isDSAttached()) {
      return LEDPatterns.ERROR;
    } else if (DriverStation.isDisabled()) {
      return GOOD;
    } else {
      return LEDPatterns.RAINBOW;
    }
  };

  private static final LEDPattern LIGHTNING = new LEDPattern() {
    int index = 0;
    int size = 0;
    boolean active = false;
    public void draw(LEDStripInterface leds) {
      leds.setAllRGB(40, 40, 100);
      if (Math.random() < .01 || active) {
        size = (int) (Math.random() * 10 + 1); 
        index = (int) (Math.random() * (leds.length() - size));
        for (int i = 0; i < size; i++) {
          leds.setRGB(index + i, 200, 200, 200);
        }
        if (Math.random() < .25) {
          active = false;
        } else {
          active = true;
        }
      }
    };
  };

  private static final Supplier<LEDPattern> MAIN_PATTERN = () -> {
    if (!DriverStation.isDSAttached()) {
      return LEDPatterns.OFF;
    } else if (DriverStation.isDisabled()) {
      return LIGHTNING;
    } else {
      return LEDPatterns.RAINBOW;
    }
  };
}