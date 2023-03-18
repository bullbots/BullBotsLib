// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * AxisTrigger triggers when the given axis is beyond the threshold in the desired direction.
 * The default direction is {@link Direction#BOTH_WAYS}, and the default threshold is 0.2.
 */
@SuppressWarnings("unused")
public class AxisTrigger extends Trigger {
    public enum Direction {
        BOTH_WAYS,
        POSITIVE_ONLY,
        NEGATIVE_ONLY
    }

    /**
     * Creates a trigger on the given controller and axis.
     * @param stick the controller
     * @param axis the axis
     */
    public AxisTrigger(GenericHID stick, int axis) {
        this(stick, axis, Direction.BOTH_WAYS);
    }

    /**
     * Creates a trigger on the given controller and axis.
     *
     * It will only activate in the given direction.
     * @param stick the controller
     * @param axis the axis
     * @param direction the direction that activates the trigger
     */
    public AxisTrigger(GenericHID stick, int axis, Direction direction) {
        this(stick, axis, direction, .2);
    }

    /**
     * Creates a trigger on the given controller and axis.
     *
     * It will only activate when the threshold is exceeded.
     * @param stick the controller
     * @param axis the axis
     * @param axisThreshold the threshold that must be passed to activate the trigger
     */
    public AxisTrigger(GenericHID stick, int axis, double axisThreshold) {
        this(stick, axis, Direction.BOTH_WAYS, axisThreshold);
    }

    /**
     * Creates a trigger on the given controller and axis.
     *
     * It will only activate when the threshold is exceeded in the given direction.
     * @param stick the controller
     * @param axis the axis
     * @param direction the direction that activates the trigger
     * @param axisThreshold the threshold that must be passed to activate the trigger
     */
    public AxisTrigger(GenericHID stick, int axis, Direction direction, double axisThreshold) {
        super(() -> switch (direction) {
            case BOTH_WAYS -> Math.abs(stick.getRawAxis(axis)) > axisThreshold;
            case POSITIVE_ONLY -> stick.getRawAxis(axis) > axisThreshold;
            case NEGATIVE_ONLY -> stick.getRawAxis(axis) < -axisThreshold;
        });
    }
}