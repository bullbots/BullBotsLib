package frc.team1891.common.logger;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.nio.file.Files;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A logger class that uses the DataLog class. Use parameters to also output to the
 * standard output or live in SmartDashboard. Log levels can be used to only output
 * entries that are at or above the level set (default is LogLevel.CRITICAL).
 *
 * Paramters
 *  boolean showInConsole   Flag to also show output in system console
 *  boolean liveOutput      Flag to also send output to SmartDashboard
 *  LogLevel level          Logging level
 * 
 * Set the log type after instantiating the logger.
 *  m_Logger = new BullLogger(true, false, BullLogger.LogLevel.DEBUG);
 *
 * Calling the logger is done by specifying the type of log, supplying the label, and optionally
 * the log level. A different lable will log under a different log entry. 
 *  m_Logger.putString("Log label", "log string", BullLogger.LogLevel.WARNING)
 * 
 *  In this example, if the log level is WARNING or greater, the log entry will be emitted.
 * 
 * If wanting driver station info to be logged, call logDriverStation().
 */
public class BullLogger {
    private DataLog m_dataLogger;
    private boolean m_ShowInConsole;
    private boolean m_liveOutput;

    public static enum LogLevel {
        DEBUG(1),
        INFO(2), 
        WARNING(3),
        ERROR(4),
        CRITICAL(5);

        public int levelNumber;

        private LogLevel(int number) {
            this.levelNumber = number;
        }

        public int getValue() {
            return this.levelNumber;
        }
    }

    private LogLevel m_LogLevel;

    private final ConcurrentMap<String, Object> m_logEntries = new ConcurrentHashMap<>();

    public BullLogger(boolean showInConsole, boolean liveOutput) {
        // default log level is CRITICAL
        this(showInConsole, liveOutput, LogLevel.CRITICAL);
    }

    public BullLogger(boolean showInConsole, boolean liveOutput, LogLevel level) {
        // save the logger settings
        m_ShowInConsole = showInConsole;
        m_liveOutput = liveOutput;

        // make sure we are using a USB directory on a real robot
        boolean useDataLogger = true;
        if (RobotBase.isReal()) {
            if (!this.loggerIsUSBDrive()) {
                // don't use the data logger
                logException("***** Logger not available: no usb drive found *****");
                m_dataLogger = null;
                useDataLogger = false;
            }
        }

        if (useDataLogger) {
            m_dataLogger = DataLogManager.getLog();
        }

       m_LogLevel = level;
    }

    private boolean loggerIsUSBDrive() {

        Path logDirTest;
        try {
            logDirTest = Paths.get("/u").toRealPath();
            
            return Files.isWritable(logDirTest);
        } catch (IOException e) {
            logException("Exception checking for usb path: " + e.getMessage());
        }

        return false;
    }

    // We don't want to throw exceptions out of the logger, so log anything that might 
    // normally be an exception as an error to the console.
    private void logException(String exception) {
        DriverStation.reportWarning(exception, true);
    }

    // Allow logging of an integer.
    public void putInteger(String label, int entry, LogLevel level) {
        IntegerLogEntry logger = null;

        // if data logging, get the entry
        if (m_dataLogger != null) {
            logger = (IntegerLogEntry) m_logEntries.putIfAbsent(label, new IntegerLogEntry(m_dataLogger, label));

            if (logger == null) {
                // get the entry just added
                logger = (IntegerLogEntry) m_logEntries.get(label);
            }
        }

        // now log it
        logEntry(logger, label, entry, level);
    }

    // Default to Critical log level, if not specified.
    public void putInteger(String label, int entry) {
        this.putInteger(label, entry, LogLevel.CRITICAL);
    }

    // Allow logging of a double.
    public void putDouble(String label, double entry, LogLevel level) {
        DoubleLogEntry logger = null;

        // if data logging, get the entry
        if (m_dataLogger != null) {
            logger = (DoubleLogEntry) m_logEntries.putIfAbsent(label, new DoubleLogEntry(m_dataLogger, label));

            if (logger == null) {
                // get the entry just added
                logger = (DoubleLogEntry) m_logEntries.get(label);
            }
        }

        // now log it
        logEntry(logger, label, entry, level);
    }

    // Default to Critical log level, if not specified.
    public void putDouble(String label, double entry) {
        this.putDouble(label, entry, LogLevel.CRITICAL);
    }

    // Allow logging of a string.
    public void putString(String label, String entry, LogLevel level) {
        StringLogEntry logger = null;

        // if data logging, get the entry
        if (m_dataLogger != null) {
            logger = (StringLogEntry) m_logEntries.putIfAbsent(label, new StringLogEntry(m_dataLogger, label));

            if (logger == null) {
                // get the entry just added
                logger = (StringLogEntry) m_logEntries.get(label);
            }
        }

        // now log it
        logEntry(logger, label, entry, level);
    }

    // Default to Critical log level, if not specified.
    public void putString(String label, String entry) {
        this.putString(label, entry, LogLevel.CRITICAL);
    }

    private void showInConsole(String entry) {
        // show in console if asked
        if (m_ShowInConsole) {
            System.out.println(entry);
        }
    }

    private void logToDataLogger(StringLogEntry logger, String entry) {
        if (m_dataLogger != null) {
            if (logger == null) {
                System.out.println("String logger not set up\n");
                return;
            }

            try {
                logger.append(entry);
            } catch (Exception e) {
                logException("Exception trying to log string '" + entry + "': " + e.getMessage());
            }
        }
    }

    private void showLiveOutput(String label, String entry) {
        if (m_liveOutput) {
            SmartDashboard.putString(label, entry);
        }
    }

    private void logEntry(StringLogEntry logger, String label, String entry, LogLevel level) {
        // only log if at the right level
        if (level.getValue() < this.m_LogLevel.getValue()) {
            return;
        }

        logToDataLogger(logger, entry);

        showInConsole(label + ": " + entry);

        showLiveOutput(label, entry);
    }

    private void logToDataLogger(IntegerLogEntry logger, int entry) {
        if (m_dataLogger != null) {
            if (logger == null) {
                System.out.println("Integer logger not set up\n");
                return;
            }

            try {
                logger.append(entry);
            } catch (Exception e) {
                logException("Exception trying to log int '" + entry + "': " + e.getMessage());
            }
        }
    }

    private void showLiveOutput(String label, int entry) {
        if (m_liveOutput) {
            SmartDashboard.putNumber(label, entry);
        }
    }

    private void logEntry(IntegerLogEntry logger, String label, int entry, LogLevel level) {
        if (level.getValue() < this.m_LogLevel.getValue()) {
            return;
        }

        logToDataLogger(logger, entry);

        showInConsole(label + ": " + entry);

        showLiveOutput(label, entry);
    }

    private void logToDataLogger(DoubleLogEntry logger, double entry) {
        if (m_dataLogger != null) {
            if (logger == null) {
                System.out.println("Double logger not set up\n");
                return;
            }

            try {
                logger.append(entry);
            } catch (Exception e) {
                logException("Exception trying to log double '" + entry + "': " + e.getMessage());
            }
        }
    }

    private void showLiveOutput(String label, double entry) {
        if (m_liveOutput) {
            SmartDashboard.putNumber(label, entry);
        }
    }

    private void logEntry(DoubleLogEntry logger, String label, double entry, LogLevel level) {
        if (level.getValue() < this.m_LogLevel.getValue()) {
            return;
        }

        logToDataLogger(logger, entry);

        showInConsole(label + ": " + entry);

        showLiveOutput(label, entry);
    }

    public void logDriverStation() {
        if (m_dataLogger != null) {
            // start data logging
            DataLogManager.start();

            // Record both DS control and joystick data
            DriverStation.startDataLog(m_dataLogger);
        }
   }
}
