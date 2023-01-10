// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Thank you to FRC team 3244.
 * 
 * AxisTrigger triggers when the given axis is beyond the threshold in the desired direction.
 * The default direction is {@link Direction#BOTH_WAYS}.
 */
public class AxisTrigger extends Trigger {
    public enum Direction {
        BOTH_WAYS,
        POSITIVE_ONLY,
        NEGATIVE_ONLY
    }
    public AxisTrigger(GenericHID stick, int axis) {
        this(stick, axis, Direction.BOTH_WAYS);
    }

    public AxisTrigger(GenericHID stick, int axis, Direction direction) {
        this(stick, axis, direction, .2);
    }
    
    public AxisTrigger(GenericHID stick, int axis, Direction direction, double axisThreshold) {
        super(() -> {
            switch (direction) {
                case BOTH_WAYS:
                    return Math.abs(stick.getRawAxis(axis)) > axisThreshold;

                case POSITIVE_ONLY:
                    return stick.getRawAxis(axis) > axisThreshold;

                case NEGATIVE_ONLY:
                    return stick.getRawAxis(axis) < -axisThreshold;
            }
            return false;
        });
    }
}