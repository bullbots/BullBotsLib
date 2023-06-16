// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1891.common.led.LEDMatrix;
import frc.team1891.common.led.LEDMatrixInterface;
import frc.team1891.common.led.LEDStrip;
import frc.team1891.common.led.LEDMatrix.LEDMatrixPattern;
import frc.team1891.common.led.LEDMatrix.LEDMatrixPatterns;

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
  
  private final LEDStrip leds;
  private final LEDMatrix matrix;
  private static final int numRows = 16;
  private static final int numCols = 16;

  private final AtomicReference<LEDMatrixPattern> ledPattern = new AtomicReference<LEDMatrixPattern>(null);
  // This Notifier acts in place of periodic, so updating the buffer will happen on a seperate thread.
  private final Notifier periodicThread;
  
  /** Initializes the LEDs subsystem. */
  private LEDs() {
    leds = new LEDStrip(9, numRows * numCols);
    matrix = new LEDMatrix(leds, 0, numRows, numCols, true);

    periodicThread = new Notifier(() -> {
      LEDMatrixPattern pattern = ledPattern.get();
      if (pattern != null) {
        pattern.run(matrix);
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

  public void setCustomPattern(LEDMatrixPattern customPattern, boolean nonLooping) {
    if (nonLooping) {
      ledPattern.set(new NonLoopingPattern(customPattern));
    } else {
      ledPattern.set(customPattern);
    }
    currentMode = null;
  }

  public void setCustomPattern(LEDMatrixPattern customPattern) {
    ledPattern.set(customPattern);
    currentMode = null;
  }

  public void setMode(LEDMode mode) {
    if (mode != currentMode) {
      currentMode = mode;
      switch (currentMode) {
        case OFF:
          ledPattern.set(LEDMatrixPatterns.OFF);
          break;
        case DISCONNECTED:
          ledPattern.set(LEDs.CRAZY);
          break;
        case DISABLED:
          ledPattern.set(new NonLoopingPattern(LEDMatrixPattern.setRGB(0, 50, 0)));
          break;
        case RAINBOW:
          ledPattern.set(LEDMatrixPatterns.RAINBOW);
          break;
      }
    }
  }

  @Override
  public void periodic() {}

  public class NonLoopingPattern implements LEDMatrixPattern {
    private final LEDMatrixPattern pattern;
    public NonLoopingPattern(LEDMatrixPattern pattern) {
      this.pattern = pattern;
    }

    @Override
    public void run(LEDMatrixInterface leds) {
      pattern.run(leds);
      ledPattern.set(null);
    }

    @Override
    public void draw(LEDMatrixInterface leds) {
      pattern.draw(leds);
    }
  }

  public static final LEDMatrixPattern CRAZY = new LEDMatrixPattern() {
    private int rainbowFirstPixelHue = 0;
    public void draw(LEDMatrixInterface leds) {
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
