package frc.robot.utility;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
 *  m_stringLogger = new Logger1891("MyLogName", true, false);
 *  m_stringLogger.setLogType(Logger1891.LogType.STRING);
 *  m_stringLogger.setLogLevel(Logger1891.LogLevel.INFO);
 *
 * Calling the logger is done by specifying the entry and optionally the log level.
 *  m_stringLogger.logEntr("log string", Logger1891.LogLevel.WARNING)
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
        // save the logger
        m_logName = logName;
        m_dataLogger = DataLogManager.getLog();
        m_ShowInConsole = showInConsole;
        m_liveOutput = liveOutput;

        // default log level is CRITICAL
        m_LogLevel = LogLevel.CRITICAL;
    }

    public void setLogLevel(LogLevel logLevel) {
        this.m_LogLevel = logLevel;
    }

    public void setLogType(LogType logType) throws Exception {
        this.m_LogType = logType;

        if (logType == LogType.STRING) {
            // set up a string logger
            m_stringLog = new StringLogEntry(m_dataLogger, this.m_logName);
        } else if (logType == LogType.INT) {
            // set up an int logger
            m_intLog = new IntegerLogEntry(m_dataLogger, this.m_logName);
        } else if (logType == LogType.DOUBLE) {
            // set up an int logger
            m_intLog = new IntegerLogEntry(m_dataLogger, this.m_logName);
        } else {
            throw new Exception("Invalid log type: " + logType);
        }
    }

    private void showInConsole(String entry) {
        // show in console if asked
        if (m_ShowInConsole) {
            System.out.print(entry);
        }
    }

    public void logEntry(String entry) {
        // default is CRITICAL level
        this.logEntry(entry, LogLevel.CRITICAL);
    }

    public void logEntry(String entry, LogLevel level) {
        if (m_stringLog == null) {
            System.out.println("String logger not set up\n");
            return;
        }

        // only log if at the right level
        if (level.getValue() < this.m_LogLevel.getValue()) {
            return;
        }

        m_stringLog.append(entry);

        showInConsole(m_logName + ": " + entry + "\n");

        // show live output, if asked
        if (m_liveOutput) {
            SmartDashboard.putString(m_logName, entry);
        }
    }

    public void logEntry(int entry) {
        // default is CRITICAL level
        logEntry(entry, LogLevel.CRITICAL);
    }

    public void logEntry(int entry, LogLevel level) {
        if (m_intLog == null) {
            System.out.println("Int logger not set up\n");
            return;
        }

        // only log if at the right level
        if (level.getValue() < this.m_LogLevel.getValue()) {
            return;
        }

        m_intLog.append(entry);

        showInConsole(m_logName + ": " + entry + "\n");

        // show live output, if asked
        if (m_liveOutput) {
            SmartDashboard.putNumber(m_logName, entry);
        }
    }

    public void logEntry(double entry) {
        // default is CRITICAL level
        logEntry(entry, LogLevel.CRITICAL);
    }

    public void logEntry(double entry, LogLevel level) {
        if (m_doubleLog == null) {
            System.out.println("Int logger not set up\n");
            return;
        }

        // only log if at the right level
        if (level.getValue() < this.m_LogLevel.getValue()) {
            return;
        }

        m_doubleLog.append(entry);

        showInConsole(m_logName + ": " + entry + "\n");

        // show live output, if asked
        if (m_liveOutput) {
            SmartDashboard.putNumber(m_logName, entry);
        }
    }

    public void logDriverStation() {
        // start data logging
        DataLogManager.start();

        // Record both DS control and joystick data
        DriverStation.startDataLog(m_dataLogger);
   }
}
