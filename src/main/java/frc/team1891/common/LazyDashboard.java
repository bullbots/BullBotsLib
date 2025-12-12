package frc.team1891.common;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A helper class to make working with SmartDashboard simpler while also adding protection against overloading
 * NetworkTables.
 * <p>When you use this class, a new {@link Notifier} will be created that updates all entries fed to
 * LazyDashboard with a certain update interval - meaning it only updates every n loops.  The default is 50.
 * The notifier runs every .02 seconds, meaning each value will update once a second, but all values will be staggered
 * from each other</p>
 */
@SuppressWarnings("unused")
public abstract class LazyDashboard {
    private static final Notifier notifier = new Notifier(LazyDashboard::updateAll);
    static {
        notifier.setName("LazyDashboard Thread");
        notifier.startPeriodic(.02);
    }
    private static final int DEFAULT_INTERVAL = 50;
    private static final ArrayList<LazyDashboard> lazyDashboards = new ArrayList<>();

    private static void updateAll() {
        for (LazyDashboard lazyDashboard : lazyDashboards) {
            lazyDashboard.update();
        }
    }

    /**
     * Changes the update interval of an existing LazyDashboard object.
     * @param name name of object
     * @param newInterval new interval to update with
     * @return true if there was an object with the given name
     */
    public static boolean changeUpdateInterval(String name, int newInterval) {
        LazyDashboard dashboard = null;
        for (LazyDashboard lazyDashboard : lazyDashboards) {
            if (lazyDashboard.entryString.equals(name)) {
                dashboard = lazyDashboard;
            }
        }
        if (dashboard == null) {
            return false;
        }
        dashboard.changeUpdateInterval(newInterval);
        return true;
    }

    /**
     * Periodically (default is every second) updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addBoolean(String name, BooleanSupplier supplier) {
        return addBoolean(name, DEFAULT_INTERVAL, supplier);
    }

    /**
     * Periodically updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param updateInterval the number of periodic loops between updates
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addBoolean(String name, int updateInterval, BooleanSupplier supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setBoolean(supplier.getAsBoolean());
            }
        };
        if (!lazyDashboards.contains(lazyDashboardObject)) {
            lazyDashboards.add(lazyDashboardObject);
            return lazyDashboardObject;
        }
        DriverStation.reportWarning("LazyDashboard was given two objects with the same entry name (\""+name+"\").  The second object was ignored", false);
        return null;
    }

    /**
     * Periodically (default is every 50ms) updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addNumber(String name, DoubleSupplier supplier) {
        return addNumber(name, DEFAULT_INTERVAL, supplier);
    }

    /**
     * Periodically updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param updateInterval the number of periodic loops between updates
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addNumber(String name, int updateInterval, DoubleSupplier supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setDouble(supplier.getAsDouble());
            }
        };
        if (!lazyDashboards.contains(lazyDashboardObject)) {
            lazyDashboards.add(lazyDashboardObject);
            return lazyDashboardObject;
        }
        DriverStation.reportWarning("LazyDashboard was given two objects with the same entry name (\""+name+"\").  The second object was ignored", false);
        return null;
    }

    /**
     * Periodically (default is every 50ms) updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addString(String name, Supplier<String> supplier) {
        return addString(name, DEFAULT_INTERVAL, supplier);
    }

    /**
     * Periodically updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param updateInterval the number of periodic loops between updates
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addString(String name, int updateInterval, Supplier<String> supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setString(supplier.get());
            }
        };
        if (!lazyDashboards.contains(lazyDashboardObject)) {
            lazyDashboards.add(lazyDashboardObject);
            return lazyDashboardObject;
        }
        DriverStation.reportWarning("LazyDashboard was given two objects with the same entry name (\""+name+"\").  The second object was ignored", false);
        return null;
    }

    /**
     * Periodically (default is every 50ms) updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addBooleanArray(String name, Supplier<Boolean[]> supplier) {
        return addBooleanArray(name, DEFAULT_INTERVAL, supplier);
    }

    /**
     * Periodically updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param updateInterval the number of periodic loops between updates
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addBooleanArray(String name, int updateInterval, Supplier<Boolean[]> supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setBooleanArray(supplier.get());
            }
        };
        if (!lazyDashboards.contains(lazyDashboardObject)) {
            lazyDashboards.add(lazyDashboardObject);
            return lazyDashboardObject;
        }
        DriverStation.reportWarning("LazyDashboard was given two objects with the same entry name (\""+name+"\").  The second object was ignored", false);
        return null;
    }

    /**
     * Periodically (default is every 50ms) updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addNumberArray(String name, Supplier<Double[]> supplier) {
        return addNumberArray(name, DEFAULT_INTERVAL, supplier);
    }

    /**
     * Periodically updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param updateInterval the number of periodic loops between updates
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addNumberArray(String name, int updateInterval, Supplier<Double[]> supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setNumberArray(supplier.get());
            }
        };
        if (!lazyDashboards.contains(lazyDashboardObject)) {
            lazyDashboards.add(lazyDashboardObject);
            return lazyDashboardObject;
        }
        DriverStation.reportWarning("LazyDashboard was given two objects with the same entry name (\""+name+"\").  The second object was ignored", false);
        return null;
    }

    /**
     * Periodically (default is every 50ms) updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addStringArray(String name, Supplier<String[]> supplier) {
        return addStringArray(name, DEFAULT_INTERVAL, supplier);
    }

    /**
     * Periodically updates a SmartDashboard value with the given supplier
     * @param name SmartDashboard value name
     * @param updateInterval the number of periodic loops between updates
     * @param supplier supplier value
     * @return the object used to periodically update SmartDashboard (this can be ignored)
     */
    public static LazyDashboard addStringArray(String name, int updateInterval, Supplier<String[]> supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setStringArray(supplier.get());
            }
        };
        if (!lazyDashboards.contains(lazyDashboardObject)) {
            lazyDashboards.add(lazyDashboardObject);
            return lazyDashboardObject;
        }
        DriverStation.reportWarning("LazyDashboard was given two objects with the same entry name (\""+name+"\").  The second object was ignored", false);
        return null;
    }


    private final String entryString;
    /** The NetworkTable entry for this dashboard item. */
    protected final NetworkTableEntry entry;
    private int count;
    private int updateInterval;

    private LazyDashboard(String name, int updateInterval) {
        this.entryString = name;
        this.entry = SmartDashboard.getEntry(name);
        this.count = (int) (Math.random()*updateInterval);
        this.updateInterval = updateInterval;
    }

    /**
     * Attempts to update the SmartDashboard value.  It will update successfully if the number of loops since the last
     * successful update is equal to the update interval.
     * @return true if the update was successful
     */
    public boolean update() {
        count++;
        if (count == updateInterval) {
            count = 0;
            publishToSmartDashboard();
            return true;
        }
        return false;
    }

    /**
     * Change the interval at which this data entry updates
     * @param newInterval the new update interval
     */
    public void changeUpdateInterval(int newInterval) {
        this.updateInterval = newInterval;
    }

    /**
     * Publish an updated value for this entry to SmartDashboard.
     */
    protected abstract void publishToSmartDashboard();

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        LazyDashboard that = (LazyDashboard) o;
        return Objects.equals(entryString, that.entryString);
    }

    @Override
    public int hashCode() {
        return Objects.hash(entry);
    }
}
