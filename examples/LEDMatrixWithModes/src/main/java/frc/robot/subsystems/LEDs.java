// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1891.common.led.LEDMatrix;
import frc.team1891.common.led.LEDMatrix.LEDPattern;
import frc.team1891.common.led.LEDMatrix.LEDPatterns;

public class LEDs extends SubsystemBase {
  private static LEDs instance = null;
  public static LEDs getInstance() {
    if (instance == null) {
      instance = new LEDs();
    }
    return instance;
  }
  
  public enum LEDMode {
    OFF,
    DISCONNECTED,
    DISABLED,
    RAINBOW
  }

  private LEDMode currentMode = LEDMode.OFF;
  
  private final LEDMatrix leds;
  private static final int numRows = 16;
  private static final int numCols = 16;

  // See https://github.com/STMARobotics/frc-7028-2023/blob/main/src/main/java/frc/robot/subsystems/LEDSubsystem.java
  private final AtomicReference<LEDPattern> ledPattern = new AtomicReference<LEDPattern>(null);
  // This Notifier acts in place of periodic, so updating the buffer will happen on a seperate thread.
  private final Notifier periodicThread;
  
  /** Creates a new LEDs. */
  public LEDs() {
    leds = new LEDMatrix(9, numRows, numCols, true);

    periodicThread = new Notifier(() -> {
      LEDPattern pattern = ledPattern.get();
      if (pattern != null) {
        pattern.run(leds);
      }
    });
    periodicThread.setName("LED periodic");
    periodicThread.startPeriodic(.02);

    setMode(LEDMode.OFF);
  }

  public void start() {
    leds.start();
  }

  public void stop() {
    leds.stop();
  }

  public void setCustomPattern(LEDPattern customPattern, boolean nonLooping) {
    if (nonLooping) {
      ledPattern.set(new NonLoopingPattern(customPattern));
    } else {
      ledPattern.set(customPattern);
    }
    currentMode = null;
  }

  public void setCustomPattern(LEDPattern customPattern) {
    ledPattern.set(customPattern);
    currentMode = null;
  }

  public void setMode(LEDMode mode) {
    if (mode != currentMode) {
      currentMode = mode;
      switch (currentMode) {
        case OFF:
          ledPattern.set(LEDPatterns.OFF);
          break;
        case DISCONNECTED:
          ledPattern.set(LEDs.CRAZY);
          break;
        case DISABLED:
          ledPattern.set(new NonLoopingPattern(LEDPattern.setRGB(0, 50, 0)));
          break;
        case RAINBOW:
          ledPattern.set(LEDPatterns.RAINBOW);
          break;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public class NonLoopingPattern implements LEDPattern {
    private final LEDPattern pattern;
    public NonLoopingPattern(LEDPattern pattern) {
      this.pattern = pattern;
    }

    @Override
    public void run(LEDMatrix leds) {
      pattern.run(leds);
      ledPattern.set(null);
    }

    @Override
    public void draw(LEDMatrix leds) {
      pattern.draw(leds);
    }
  }

  public static final LEDPattern CRAZY = new LEDPattern() {
    private int rainbowFirstPixelHue = 0;
    public void draw(LEDMatrix leds) {
        for (int i = 0; i < numRows; i++) {
            final int rowStartHue = (rainbowFirstPixelHue + (i * 180 / (2 * numRows))) % 180;
            for (int j = 0; j < numCols; j++) {
                final int hue = (rowStartHue + (j * 180 / (2 * numCols))) % 180;
                leds.setHSV(i * numCols + j, hue, 255, 128);
            }
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue++;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }
  };
}
