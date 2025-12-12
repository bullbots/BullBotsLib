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
    /** Controller POV directions */
    public enum POV {
        /** North (0 degrees). */
        NORTH(0),
        /** Northeast (45 degrees). */
        NORTHEAST(45),
        /** East (90 degrees). */
        EAST(90),
        /** Southeast (135 degrees). */
        SOUTHEAST(135),
        /** South (180 degrees). */
        SOUTH(180),
        /** Southwest (225 degrees). */
        SOUTHWEST(225),
        /** West (270 degrees). */
        WEST(270),
        /** Northwest (315 degrees). */
        NORTHWEST(315),

        /** No POV pressed. */
        NONE(-1);

        private final int value;
        POV(int value) {
            this.value = value;
        }

        /**
         * Gets the numerical value of this POV direction.
         * @return the POV value in degrees
         */
        public int getValue() {
            return value;
        }
    }

    /**
     * Constructs a POVTrigger for the specified joystick and POV direction.
     * @param joystick the joystick to monitor
     * @param pov the POV direction to trigger on
     */
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

    /**
     * Simplifing POVm allowing triggers to work when two directions are pressed at the same time (a diagonal POV).
     */
    public enum Direction {
        /** Up direction. */
        UP(POV.NORTH),
        /** Down direction. */
        DOWN(POV.SOUTH),
        /** Left direction. */
        LEFT(POV.WEST),
        /** Right direction. */
        RIGHT(POV.EAST);

        private POV pov;

        Direction(POV pov) {
            this.pov = pov;
        }

        /**
         * Returns the {@link Direction} corresponding to the given {@link POV}.
         *
         * <p>Returns null if a diagonal POV is given</p>
         * @param pov the POV to convert
         * @return the corresponding direction, or null if diagonal
         */
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
