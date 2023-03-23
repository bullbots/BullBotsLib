package frc.team1891.common.logger;


import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * A logger class that uses the DataLog class. Use parameters to also output to the
 * standard output or live in SmartDashboard. Log levels can be used to only output
 * entries that are at or above the level set (default is LogLevel.CRITICAL).
 *
 * Parameters
 *  String logName          Name of log; also prefix for console
 *  boolean showInConsole   Flag to also show output in system console
 *  boolean liveOutput      Flag to also send output to SmartDashboard
 * 
 * Set the log type after instantiating the logger.
 *  m_stringLogger = new BullLogger("MyLogName", true, false);
 *  m_stringLogger.setLogType(BullLogger.LogType.STRING);
 *  m_stringLogger.setLogLevel(BullLogger.LogLevel.INFO);
 *
 * Calling the logger is done by specifying the entry and optionally the log level.
 *  m_stringLogger.logEntry("log string", BullLogger.LogLevel.WARNING)
 * 
 *  In this example, if the log level is WARNING or greater, the log entry will be emitted.
 * 
 * If wanting driver station info to be logged, call logDriverStation().
 */
@SuppressWarnings("unused")
public class BullLogger {
    private DataLog dataLogger;
    private final boolean showInConsole;
    private final boolean liveOutput;

    private StringLogEntry stringLog;
    private IntegerLogEntry intLog;
    private DoubleLogEntry doubleLog;

    private final String logName;

    public enum LogType {STRING, INT, DOUBLE}
    public enum LogLevel {
        DEBUG(1),
        INFO(2), 
        WARNING(3),
        ERROR(4),
        CRITICAL(5);

        public int levelNumber;

        LogLevel(int number) {
            this.levelNumber = number;
        }

        public int getValue() {
            return this.levelNumber;
        }
    }

    private LogLevel logLevel;
    private LogType logType;

    public BullLogger(String logName, boolean showInConsole, boolean liveOutput, boolean logInSim) {
        // save the logger settings
        this.logName = logName;
        this.showInConsole = showInConsole;
        this.liveOutput = liveOutput;

        // make sure we are using a USB directory when on a real robot
        boolean useDataLogger = RobotBase.isReal() || logInSim;
        if (RobotBase.isReal()) {
            if (!this.loggerIsUSBDrive()) {
                // don't use the data logger
                logException("***** Logger not available: no usb drive found *****");
                dataLogger = null;
                useDataLogger = false;
            }
        }

        if (useDataLogger) {
            dataLogger = DataLogManager.getLog();
        }

        // default log level is CRITICAL
        logLevel = LogLevel.CRITICAL;
    }

    private boolean loggerIsUSBDrive() {

        Path logDirTest;
        try {
            logDirTest = Paths.get("/u").toRealPath();
            
            String test2 = logDirTest.toString();

            return Files.isWritable(logDirTest);
        } catch (IOException e) {
            // TODO Auto-generated catch block
        }

        return false;
    }

    // We don't want to throw exceptions out of the logger, so log anything that might 
    // normally be an exception as an error to the console.
    private void logException(String exception) {
        DriverStation.reportWarning(exception, true);
    }

    public void setLogLevel(LogLevel logLevel) {
        this.logLevel = logLevel;
    }

    public void setLogType(LogType logType) {
        this.logType = logType;

        if (dataLogger == null) {
            logException("***** Logger not available *****");
        } else {
            if (logType == LogType.STRING) {
                // set up a string logger
                stringLog = new StringLogEntry(dataLogger, this.logName);
            } else if (logType == LogType.INT) {
                // set up an int logger
                intLog = new IntegerLogEntry(dataLogger, this.logName);
            } else if (logType == LogType.DOUBLE) {
                // set up an int logger
                doubleLog = new DoubleLogEntry(dataLogger, this.logName);
            } else {
                logException("Invalid log type: " + logType);
            }
        }
    }

    private void showInConsole(String entry) {
        // show in console if asked
        if (showInConsole) {
            System.out.println(entry);
        }
    }

    private void logToDataLogger(String entry) {
        if (dataLogger != null) {
            if (stringLog == null) {
                System.out.println("String logger not set up\n");
                return;
            }

            try {
                stringLog.append(entry);
            } catch (Exception e) {
                System.out.println("Exception trying to log string: " + entry);
            }
        }
    }

    private void showLiveOutput(String entry) {
        if (liveOutput) {
            SmartDashboard.putString(logName, entry);
        }
    }

    public void logEntry(String entry) {
        // default is CRITICAL level
        this.logEntry(entry, LogLevel.CRITICAL);
    }

    public void logEntry(String entry, LogLevel level) {
        // only log if at the right level
        if (level.getValue() < this.logLevel.getValue()) {
            return;
        }

        logToDataLogger(entry);

        showInConsole(logName + ": " + entry);

        showLiveOutput(entry);
    }

    private void logToDataLogger(int entry) {
        if (dataLogger != null) {
            if (intLog == null) {
                System.out.println("Integer logger not set up\n");
                return;
            }

            try {
                intLog.append(entry);
            } catch (Exception e) {
                System.out.println("Exception trying to log int: " + entry);
            }
        }
    }

    private void showLiveOutput(int entry) {
        if (liveOutput) {
            SmartDashboard.putNumber(logName, entry);
        }
    }

    public void logEntry(int entry) {
        // default is CRITICAL level
        logEntry(entry, LogLevel.CRITICAL);
    }

    public void logEntry(int entry, LogLevel level) {
        if (level.getValue() < this.logLevel.getValue()) {
            return;
        }

        logToDataLogger(entry);

        showInConsole(logName + ": " + entry);

        showLiveOutput(entry);
    }

    private void logToDataLogger(double entry) {
        if (dataLogger != null) {
            if (doubleLog == null) {
                System.out.println("Double logger not set up\n");
                return;
            }

            try {
                doubleLog.append(entry);
            } catch (Exception e) {
                System.out.println("Exception trying to log double: " + entry);
            }
        }
    }

    private void showLiveOutput(double entry) {
        if (liveOutput) {
            SmartDashboard.putNumber(logName, entry);
        }
    }

    public void logEntry(double entry) {
        // default is CRITICAL level
        logEntry(entry, LogLevel.CRITICAL);
    }

    public void logEntry(double entry, LogLevel level) {
        if (level.getValue() < this.logLevel.getValue()) {
            return;
        }

        logToDataLogger(entry);

        showInConsole(logName + ": " + entry);

        showLiveOutput(entry);
    }

    public void logDriverStation() {
        if (dataLogger != null) {
            // start data logging
            DataLogManager.start();

            // Record both DS control and joystick data
            DriverStation.startDataLog(dataLogger);
        }
   }
}
