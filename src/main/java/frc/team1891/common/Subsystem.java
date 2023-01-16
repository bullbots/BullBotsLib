package frc.team1891.common;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {
    protected final ShuffleboardTab shuffleboardTab;
    public Subsystem(ShuffleboardTab shuffleboardTab) {
        this.shuffleboardTab = shuffleboardTab;
    }

    protected abstract void configureShuffleboard();
}
