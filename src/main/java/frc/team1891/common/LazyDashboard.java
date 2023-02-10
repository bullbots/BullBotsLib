package frc.team1891.common;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public abstract class LazyDashboard {
    private static int DEFAULT_INTERVAL = 50;
    private static ArrayList<LazyDashboard> lazyDashboards = new ArrayList<>();

    /**
     * This should be called in robotPeriodic or some other periodic method.
     */
    public static void updateAll() {
        for (LazyDashboard lazyDashboard : lazyDashboards) {
            lazyDashboard.update();
        }
    }

    public static LazyDashboard addBoolean(String name, BooleanSupplier supplier) {
        return addBoolean(name, DEFAULT_INTERVAL, supplier);
    }

    public static LazyDashboard addBoolean(String name, int updateInterval, BooleanSupplier supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setBoolean(supplier.getAsBoolean());
            }
        };
        lazyDashboards.add(lazyDashboardObject);
        return lazyDashboardObject;
    }

    public static LazyDashboard addNumber(String name, DoubleSupplier supplier) {
        return addNumber(name, DEFAULT_INTERVAL, supplier);
    }

    public static LazyDashboard addNumber(String name, int updateInterval, DoubleSupplier supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setDouble(supplier.getAsDouble());
            }
        };
        lazyDashboards.add(lazyDashboardObject);
        return lazyDashboardObject;
    }

    public static LazyDashboard addString(String name, Supplier<String> supplier) {
        return addString(name, DEFAULT_INTERVAL, supplier);
    }

    public static LazyDashboard addString(String name, int updateInterval, Supplier<String> supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setString(supplier.get());
            }
        };
        lazyDashboards.add(lazyDashboardObject);
        return lazyDashboardObject;
    }

    public static LazyDashboard addBooleanArray(String name, Supplier<Boolean[]> supplier) {
        return addBooleanArray(name, DEFAULT_INTERVAL, supplier);
    }

    public static LazyDashboard addBooleanArray(String name, int updateInterval, Supplier<Boolean[]> supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setBooleanArray(supplier.get());
            }
        };
        lazyDashboards.add(lazyDashboardObject);
        return lazyDashboardObject;
    }

    public static LazyDashboard addNumberArray(String name, Supplier<Double[]> supplier) {
        return addNumberArray(name, DEFAULT_INTERVAL, supplier);
    }

    public static LazyDashboard addNumberArray(String name, int updateInterval, Supplier<Double[]> supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setNumberArray(supplier.get());
            }
        };
        lazyDashboards.add(lazyDashboardObject);
        return lazyDashboardObject;
    }

    public static LazyDashboard addStringArray(String name, Supplier<String[]> supplier) {
        return addStringArray(name, DEFAULT_INTERVAL, supplier);
    }

    public static LazyDashboard addStringArray(String name, int updateInterval, Supplier<String[]> supplier) {
        LazyDashboard lazyDashboardObject = new LazyDashboard(name, updateInterval) {
            @Override
            public void publishToSmartDashboard() {
                entry.setStringArray(supplier.get());
            }
        };
        lazyDashboards.add(lazyDashboardObject);
        return lazyDashboardObject;
    }


    protected final NetworkTableEntry entry;
    private int count;
    private final int updateInterval;

    private LazyDashboard(String name, int updateInterval) {
        this.entry = SmartDashboard.getEntry(name);
        this.count = (int) (Math.random()*updateInterval);
        this.updateInterval = updateInterval;
    }

    public void update() {
        count++;
        if (count > updateInterval) {
            count = 0;
            publishToSmartDashboard();
        }
    }

    protected abstract void publishToSmartDashboard();
}
