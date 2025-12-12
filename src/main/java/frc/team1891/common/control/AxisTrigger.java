// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

/**
 * AxisTrigger triggers when the given axis is beyond the threshold in the desired direction.
 * The default direction is {@link Direction#BOTH_WAYS}, and the default threshold is 0.2.
 */
@SuppressWarnings("unused")
public class AxisTrigger extends Trigger {
    private static final double DEFAULT_THRESHOLD = .2;
    private static final Direction DEFAULT_DIRECTION = Direction.BOTH_WAYS;

    /**
     * The direction the axis must move in order to trigger.
     */
    public enum Direction {
        /** Trigger activates in both positive and negative directions. */
        BOTH_WAYS,
        /** Trigger activates only in the positive direction. */
        POSITIVE_ONLY,
        /** Trigger activates only in the negative direction. */
        NEGATIVE_ONLY
    }

    /**
     * Creates a trigger on the given controller and axis.
     * @param stick the controller
     * @param axis the axis
     */
    public AxisTrigger(GenericHID stick, int axis) {
        this(stick, axis, DEFAULT_DIRECTION);
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
        this(stick, axis, direction, DEFAULT_THRESHOLD);
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
        this(stick, axis, DEFAULT_DIRECTION, axisThreshold);
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
        this(() -> stick.getRawAxis(axis), direction, axisThreshold);
    }

    /**
     * Creates a trigger on the given {@link DoubleSupplier}.
     *
     * @param axis the axis
     */
    public AxisTrigger(DoubleSupplier axis) {
        this(axis, DEFAULT_DIRECTION);
    }

    /**
     * Creates a trigger on the given {@link DoubleSupplier}.
     *
     * It will only activate when the threshold is exceeded in the given direction.
     * @param axis the axis
     * @param direction the direction that activates the trigger
     */
    public AxisTrigger(DoubleSupplier axis, Direction direction) {
        this(axis, direction, DEFAULT_THRESHOLD);
    }

    /**
     * Creates a trigger on the given {@link DoubleSupplier}.
     *
     * It will only activate when the threshold is exceeded.
     * @param axis the axis
     * @param axisThreshold the threshold that must be passed to activate the trigger
     */
    public AxisTrigger(DoubleSupplier axis, double axisThreshold) {
        this(axis, DEFAULT_DIRECTION, axisThreshold);
    }

    /**
     * Creates a trigger on the given {@link DoubleSupplier}.
     *
     * It will only activate when the threshold is exceeded in the given direction.
     * @param axis the axis
     * @param direction the direction that activates the trigger
     * @param axisThreshold the threshold that must be passed to activate the trigger
     */
    public AxisTrigger(DoubleSupplier axis, Direction direction, double axisThreshold) {
        super(() -> switch (direction) {
            case BOTH_WAYS -> Math.abs(axis.getAsDouble()) > axisThreshold;
            case POSITIVE_ONLY -> axis.getAsDouble() > axisThreshold;
            case NEGATIVE_ONLY -> axis.getAsDouble() < -axisThreshold;
        });
    }
}