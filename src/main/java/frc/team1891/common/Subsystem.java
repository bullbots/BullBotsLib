package frc.team1891.common;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A very simple base class that wraps {@link SubsystemBase} with helpful functionality a subsystem should have.
 */
public abstract class Subsystem extends SubsystemBase {
    /**
     * If a subsystem has values that should be posted to SmartDashboard, here is the place to do that.
     * <p>This is especially helpful if using {@link LazyDashboard}.</p>
     */
    protected abstract void configureSmartDashboard();

    /**
     * Stops all moving parts.
     */
    public abstract void stop();
}
