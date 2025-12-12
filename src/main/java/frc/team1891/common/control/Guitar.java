// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.control;

import edu.wpi.first.wpilibj.Joystick;

/** Control your robot with a Guitar Hero guitar. */
@SuppressWarnings("unused")
public class Guitar extends Joystick {
    /** Button ID for the green fret button. */
    public static final int GREEN = 1;
    /** Button ID for the red fret button. */
    public static final int RED = 2;
    /** Button ID for the yellow fret button. */
    public static final int YELLOW = 4;
    /** Button ID for the blue fret button. */
    public static final int BLUE = 3;
    /** Button ID for the orange fret button. */
    public static final int ORANGE = 5;
    /** Button ID for the back button. */
    public static final int BACK = 7;
    /** Button ID for the start button. */
    public static final int START = 8;

    /**
     * Constructs a Guitar controller on the specified port.
     * @param port the USB port the guitar controller is connected to
     */
    public Guitar(final int port) {
        super(port);
    }

    /**
     * Gets the state of the green fret button.
     * @return true if the green button is pressed
     */
    public boolean getGreenButton() {
        return getRawButton(GREEN);
    }

    /**
     * Gets the state of the red fret button.
     * @return true if the red button is pressed
     */
    public boolean getRedButton() {
        return getRawButton(RED);
    }

    /**
     * Gets the state of the yellow fret button.
     * @return true if the yellow button is pressed
     */
    public boolean getYellowButton() {
        return getRawButton(YELLOW);
    }

    /**
     * Gets the state of the blue fret button.
     * @return true if the blue button is pressed
     */
    public boolean getBlueButton() {
        return getRawButton(BLUE);
    }

    /**
     * Gets the state of the orange fret button.
     * @return true if the orange button is pressed
     */
    public boolean getOrangeButton() {
        return getRawButton(ORANGE);
    }

    /**
     * Gets the state of the back button.
     * @return true if the back button is pressed
     */
    public boolean getBackButton() {
        return getRawButton(BACK);
    }

    /**
     * Gets the state of the start button.
     * @return true if the start button is pressed
     */
    public boolean getStartButton() {
        return getRawButton(START);
    }

    /**
     * Gets whether the strum bar is strummed up.
     * @return true if the strum bar is in the up position
     */
    public boolean getStrumUp() {
        return getPOV() == 0;
    }

    /**
     * Gets whether the strum bar is strummed down.
     * @return true if the strum bar is in the down position
     */
    public boolean getStrumDown() {
        return getPOV() == 180;
    }

    /**
     * Gets the whammy bar axis value.
     * @return the whammy bar axis value
     */
    public double getAxis() {
        // TODO: Not sure if 5 is the correct number.
        return getRawAxis(5);
    }
}
