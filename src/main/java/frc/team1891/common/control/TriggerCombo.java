// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.control;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** TriggerCombo triggers when the given combination of conditions is satisfied. */
public class TriggerCombo extends Trigger {
    public enum TriggerComboRelationship {
        AND,
        OR,
        AND_NOT
    }

    public TriggerCombo(Trigger button1, Trigger button2, TriggerComboRelationship relationship) {
        super(() -> {
            switch (relationship) {
                case AND:
                    return button1.getAsBoolean() && button2.getAsBoolean();
                case OR:
                    return button1.getAsBoolean() || button2.getAsBoolean();
                case AND_NOT:
                    return button1.getAsBoolean() && !button2.getAsBoolean();
            }
            return false;
        });
    }
}
