// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

/**
 * Button triggers when the given POV is active.
 */
@SuppressWarnings("unused")
public class POVTrigger extends Trigger {
    public enum POV {
        NORTH(0),
        NORTHEAST(45),
        EAST(90),
        SOUTHEAST(135),
        SOUTH(180),
        SOUTHWEST(225),
        WEST(270),
        NORTHWEST(315),

        NONE(-1);

        private final int value;
        POV(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }
    
    public POVTrigger(GenericHID joystick, POV pov) {
        super(() -> (joystick.getPOV() == pov.getValue()));
    }

    private POVTrigger(BooleanSupplier condition) {
        super(condition);
    }

    /**
     * Creates a new trigger that activates if any POV is pressed.
     * @param joystick the joystick to listen to
     * @return the trigger
     */
    public static POVTrigger anyPOV(GenericHID joystick) {
        return new POVTrigger(
            () -> (joystick.getPOV() != POV.NONE.getValue())
        );
    }

    public enum Direction {
        UP(POV.NORTH),
        DOWN(POV.SOUTH),
        LEFT(POV.WEST),
        RIGHT(POV.EAST);

        private POV pov;

        Direction(POV pov) {
            this.pov = pov;
        }

        public static Direction fromPOV(POV pov) {
            for(Direction direction: Direction.values()) {
                if(direction.pov == pov) {
                    return direction;
                }
            }
            return null; // not found
        }
    }

    /**
     * Creates a new trigger to turn a POV into a set of four buttons.
     *
     * For example, UP will trigger if the POV is NORTHWEST, NORTH, or NORTHEAST.
     * @param joystick the controller to listen to
     * @param direction the direction that will activate the trigger
     * @return the trigger
     */
    public static POVTrigger asButton(GenericHID joystick, Direction direction) {
        if (direction.equals(Direction.UP)) {
            return new POVTrigger(() -> (joystick.getPOV() == POV.NORTHEAST.getValue() ||
                    joystick.getPOV() == POV.NORTH.getValue() ||
                    joystick.getPOV() == POV.NORTHWEST.getValue()));
        } else if (direction.equals(Direction.DOWN)) {
            return new POVTrigger(() -> (joystick.getPOV() == POV.SOUTHEAST.getValue() ||
                    joystick.getPOV() == POV.SOUTH.getValue() ||
                    joystick.getPOV() == POV.SOUTHWEST.getValue()));
        } else if (direction.equals(Direction.LEFT)) {
            return new POVTrigger(() -> (joystick.getPOV() == POV.NORTHWEST.getValue() ||
                    joystick.getPOV() == POV.WEST.getValue() ||
                    joystick.getPOV() == POV.SOUTHWEST.getValue()));
        } else if (direction.equals(Direction.RIGHT)) {
            return new POVTrigger(() -> (joystick.getPOV() == POV.NORTHEAST.getValue() ||
                    joystick.getPOV() == POV.EAST.getValue() ||
                    joystick.getPOV() == POV.SOUTHEAST.getValue()));
        }
        return null;
    }
}
