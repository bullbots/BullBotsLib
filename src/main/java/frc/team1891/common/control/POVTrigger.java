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
public class POVTrigger extends Trigger {
    public static enum POV {
        NORTH(0),
        NORTHEAST(45),
        EAST(90),
        SOUTHEAST(135),
        SOUTH(180),
        SOUTHWEST(225),
        WEST(270),
        NORTHWEST(315),

        NONE(-1);

        private int value;
        private POV(int value) {
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

    public static POVTrigger anyPOV(GenericHID joystick) {
        return new POVTrigger(
            () -> (joystick.getPOV() != POV.NONE.getValue())
        );
    }
}
