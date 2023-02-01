package frc.team1891.common;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public abstract class LazyDashboard {
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
        return addBoolean(name, 20, supplier);
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
        return addNumber(name, 50, supplier);
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
        return addString(name, 50, supplier);
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
        return addBooleanArray(name, 50, supplier);
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
        return addNumberArray(name, 50, supplier);
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
        return addStringArray(name, 50, supplier);
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

//    public void update(boolean bool) {
//        count++;
//        if (count > updateInterval) {
//            count = 0;
//            entry.setBoolean(bool);
//        }
//    }
//
//    public boolean getBoolean(boolean defaultValue) {
//        return entry.getBoolean(defaultValue);
//    }
//
//    public void update(double number) {
//        count++;
//        if (count > updateInterval) {
//            count = 0;
//            entry.setNumber(number);
//        }
//    }
//
//    public double getNumber(double defaultValue) {
//        return entry.getDouble(defaultValue);
//    }
//
//    public void update(String string) {
//        count++;
//        if (count > updateInterval) {
//            count = 0;
//            entry.setString(string);
//        }
//    }
//
//    public String getNumber(String defaultValue) {
//        return entry.getString(defaultValue);
//    }
//
//    public void update(boolean[] booleanArray) {
//        count++;
//        if (count > updateInterval) {
//            count = 0;
//            entry.setBooleanArray(booleanArray);
//        }
//    }
//    public void update(Boolean[] booleanArray) {
//        count++;
//        if (count > updateInterval) {
//            count = 0;
//            entry.setBooleanArray(booleanArray);
//        }
//    }
//
//    public boolean[] getBooleanArray(boolean[] defaultValue) {
//        return entry.getBooleanArray(defaultValue);
//    }
//    public Boolean[] getBooleanArray(Boolean[] defaultValue) {
//        return entry.getBooleanArray(defaultValue);
//    }
//
//    public void update(double[] numberArray) {
//        count++;
//        if (count > updateInterval) {
//            count = 0;
//            entry.setDoubleArray(numberArray);
//        }
//    }
//    public void update(Double[] numberArray) {
//        count++;
//        if (count > updateInterval) {
//            count = 0;
//            entry.setDoubleArray(numberArray);
//        }
//    }
//
//    public double[] getNumberArray(double[] defaultValue) {
//        return entry.getDoubleArray(defaultValue);
//    }
//    public Double[] getNumberArray(Double[] defaultValue) {
//        return entry.getDoubleArray(defaultValue);
//    }
//
//    public void update(String[] numberArray) {
//        count++;
//        if (count > updateInterval) {
//            count = 0;
//            entry.setStringArray(numberArray);
//        }
//    }
//
//    public String[] getStringArray(String[] defaultValue) {
//        return entry.getStringArray(defaultValue);
//    }
}
