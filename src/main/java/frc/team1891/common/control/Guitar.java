// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.control;

import edu.wpi.first.wpilibj.Joystick;

/** Control your robot with a Guitar Hero guitar. */
public class Guitar extends Joystick {
    public static final int GREEN = 1;
    public static final int RED = 2;
    public static final int YELLOW = 4;
    public static final int BLUE = 3;
    public static final int ORANGE = 5;
    public static final int BACK = 7;
    public static final int START = 8;

    public Guitar(final int port) {
        super(port);
    }

    public boolean getGreenButton() {
        return getRawButton(GREEN);
    }

    public boolean getRedButton() {
        return getRawButton(RED);
    }
    
    public boolean getYellowButton() {
        return getRawButton(YELLOW);
    }

    public boolean getBlueButton() {
        return getRawButton(BLUE);
    }

    public boolean getOrangeButton() {
        return getRawButton(ORANGE);
    }

    public boolean getBackButton() {
        return getRawButton(BACK);
    }
    
    public boolean getStartButton() {
        return getRawButton(START);
    }

    public boolean getStrumUp() {
        return getPOV() == 0;
    }

    public boolean getStrumDown() {
        return getPOV() == 180;
    }

    public double getAxis() {
        // TODO: Not sure if 5 is the correct number.
        return getRawAxis(5);
    }
}
