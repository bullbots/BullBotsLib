package frc.team1891.common.logger;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.Files;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * A logger class that uses the DataLog class. Use parameters to also output to the
 * standard output or live in SmartDashboard. Log levels can be used to only output
 * entries that are at or above the level set (default is LogLevel.CRITICAL).
 *
 * Paramters
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
public class BullLogger {
    private DataLog m_dataLogger;
    private boolean m_ShowInConsole;
    private boolean m_liveOutput;

    private StringLogEntry m_stringLog;
    private IntegerLogEntry m_intLog;
    private DoubleLogEntry m_doubleLog;

    private String m_logName;

    public static enum LogType {STRING, INT, DOUBLE}
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
    private LogType m_LogType;

    public BullLogger(String logName, boolean showInConsole, boolean liveOutput) {
        // save the logger settings
        m_logName = logName;
        m_ShowInConsole = showInConsole;
        m_liveOutput = liveOutput;

        // make sure we are using a USB directory on a real robot
        boolean useDataLogger = true;
        if (Robot.isReal()) {
            if (!this.loggerIsUSBDrive()) {
                // don't use the data logger
                LogException("***** Logger not available: no usb drive found *****");
                m_dataLogger = null;
                useDataLogger = false;
            }
        }

        if (useDataLogger) {
            m_dataLogger = DataLogManager.getLog();
        }

        // default log level is CRITICAL
        m_LogLevel = LogLevel.CRITICAL;
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
    private void LogException(String exception) {
        System.out.println(exception);
    }

    public void setLogLevel(LogLevel logLevel) {
        this.m_LogLevel = logLevel;
    }

    public void setLogType(LogType logType) {
        this.m_LogType = logType;

        if (m_dataLogger == null) {
            LogException("***** Logger not available *****");
        } else {
            if (logType == LogType.STRING) {
                // set up a string logger
                m_stringLog = new StringLogEntry(m_dataLogger, this.m_logName);
            } else if (logType == LogType.INT) {
                // set up an int logger
                m_intLog = new IntegerLogEntry(m_dataLogger, this.m_logName);
            } else if (logType == LogType.DOUBLE) {
                // set up an int logger
                m_doubleLog = new DoubleLogEntry(m_dataLogger, this.m_logName);
            } else {
                LogException("Invalid log type: " + logType);
            }
        }
    }

    private void showInConsole(String entry) {
        // show in console if asked
        if (m_ShowInConsole) {
            System.out.println(entry);
        }
    }

    private void logToDataLogger(String entry) {
        if (m_dataLogger != null) {
            if (m_stringLog == null) {
                System.out.println("String logger not set up\n");
                return;
            }

            try {
                m_stringLog.append(entry);
            } catch (Exception e) {
                System.out.println("Exception trying to log string: " + entry);
            }
        }
    }

    private void showLiveOutput(String entry) {
        if (m_liveOutput) {
            SmartDashboard.putString(m_logName, entry);
        }
    }

    public void logEntry(String entry) {
        // default is CRITICAL level
        this.logEntry(entry, LogLevel.CRITICAL);
    }

    public void logEntry(String entry, LogLevel level) {
        // only log if at the right level
        if (level.getValue() < this.m_LogLevel.getValue()) {
            return;
        }

        logToDataLogger(entry);

        showInConsole(m_logName + ": " + entry);

        showLiveOutput(entry);
    }

    private void logToDataLogger(int entry) {
        if (m_dataLogger != null) {
            if (m_intLog == null) {
                System.out.println("Integer logger not set up\n");
                return;
            }

            try {
                m_intLog.append(entry);
            } catch (Exception e) {
                System.out.println("Exception trying to log int: " + entry);
            }
        }
    }

    private void showLiveOutput(int entry) {
        if (m_liveOutput) {
            SmartDashboard.putNumber(m_logName, entry);
        }
    }

    public void logEntry(int entry) {
        // default is CRITICAL level
        logEntry(entry, LogLevel.CRITICAL);
    }

    public void logEntry(int entry, LogLevel level) {
        if (level.getValue() < this.m_LogLevel.getValue()) {
            return;
        }

        logToDataLogger(entry);

        showInConsole(m_logName + ": " + entry);

        showLiveOutput(entry);
    }

    private void logToDataLogger(double entry) {
        if (m_dataLogger != null) {
            if (m_doubleLog == null) {
                System.out.println("Double logger not set up\n");
                return;
            }

            try {
                m_doubleLog.append(entry);
            } catch (Exception e) {
                System.out.println("Exception trying to log double: " + entry);
            }
        }
    }

    private void showLiveOutput(double entry) {
        if (m_liveOutput) {
            SmartDashboard.putNumber(m_logName, entry);
        }
    }

    public void logEntry(double entry) {
        // default is CRITICAL level
        logEntry(entry, LogLevel.CRITICAL);
    }

    public void logEntry(double entry, LogLevel level) {
        if (level.getValue() < this.m_LogLevel.getValue()) {
            return;
        }

        logToDataLogger(entry);

        showInConsole(m_logName + ": " + entry);

        showLiveOutput(entry);
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
