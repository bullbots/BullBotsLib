// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1891.common.led.LEDStrip;
import frc.team1891.common.led.LEDStrip.LEDPattern;
import frc.team1891.common.led.LEDStripSegment;

public class LEDs extends SubsystemBase {
  private static LEDs instance = null;
  public static LEDs getInstance() {
    if (instance == null) {
      instance = new LEDs();
    }
    return instance;
  }

  public enum Segment {
    STATUS,
    MAIN
  }
  
  private final LEDStrip leds;
  private final LEDStripSegment statusSegment, mainSegment;
  private static final int length = 150, statusLength = 30, mainLength = 120;

  // A little hard to read, but this allows for thread same control over the LEDPatterns, and also allows for pattern suppliers
  // Pattern suppliers (such as STATUS_PATTERN inside LEDDefaultCommand.java) allow for different patterns to run with
  // certain conditions, and still ensures that if the same pattern is provided by both segments the the two segments
  // blend together.
  private final AtomicReference<Supplier<LEDPattern>> statusPattern = new AtomicReference<Supplier<LEDPattern>>(() -> null);
  private final AtomicReference<Supplier<LEDPattern>> mainPattern = new AtomicReference<Supplier<LEDPattern>>(() -> null);
  private final Notifier periodicThread;
  
  /** Creates a new LEDs. */
  public LEDs() {
    leds = new LEDStrip(9, length);

    statusSegment = new LEDStripSegment(leds, 0, statusLength);
    mainSegment = new LEDStripSegment(leds, statusLength, mainLength);

    periodicThread = new Notifier(() -> {
      LEDPattern status = statusPattern.get().get();
      LEDPattern main = mainPattern.get().get();
      if (status != null && status.equals(main)) {
        status.run(leds);
      } else {
        if (status != null) {
          status.run(statusSegment);
        }
        if (main != null) {
          main.run(mainSegment);
        }
      }
    });
    periodicThread.setName("LED periodic");
    periodicThread.startPeriodic(.02);
  }

  public void start() {
    leds.start();
  }

  public void stop() {
    leds.stop();
  }

  public void setPattern(LEDPattern customPattern) {
    statusPattern.set(() -> customPattern);
    mainPattern.set(() -> customPattern);
  }

  public void setPattern(Segment segment, LEDPattern customPattern) {
    switch (segment) {
      case STATUS:
        statusPattern.set(() -> customPattern);
        break;
      case MAIN:
        mainPattern.set(() -> customPattern);
        break;
    }
  }

  public void setPattern(Supplier<LEDPattern> customPattern) {
    statusPattern.set(customPattern);
    mainPattern.set(customPattern);
  }

  public void setPattern(Segment segment, Supplier<LEDPattern> customPattern) {
    switch (segment) {
      case STATUS:
        statusPattern.set(customPattern);
        break;
      case MAIN:
        mainPattern.set(customPattern);
        break;
    }
  }

  @Override
  public void periodic() {}
}
