/* Lime Light Docs: http://docs.limelightvision.io/en/latest/networktables_api.html# */
/* StuyPulse 694, Stuyvesant Highschool, NY */

package frc.team1891.common.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Thank you to
 * https://github.com/StuyPulse/Alfred/blob/main/src/main/java/frc/util/Limelight.java
 */
@SuppressWarnings("unused")
public class Limelight {
    private final NetworkTable table;

    // Toggle for posting to SmartDashboard
    public final boolean POST_TO_SMART_DASHBOARD = false;

    // Uses network tables to check status of limelight
    private final NetworkTableEntry timingTestEntry;
    private boolean timingTestEntryValue = false;
    public final long MAX_UPDATE_TIME = 200_000; // Micro Seconds = 0.2 Seconds

    public Limelight() {
        this("limelight");
    }

    public Limelight(String tableName) {
        // Network Table used to contact LimeLight
        NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
        table = tableInstance.getTable(tableName);
        validTargetEntry = table.getEntry("tv");
        xAngleEntry = table.getEntry("tx");
        yAngleEntry = table.getEntry("ty");
        targetAreaEntry = table.getEntry("ta");
        targetSkewEntry = table.getEntry("ts");
        latencyEntry = table.getEntry("tl");
        shortestSideLengthEntry = table.getEntry("tshort");
        longestSideLengthEntry = table.getEntry("tlong");
        horizontalSideLengthEntry = table.getEntry("thor");
        verticalSideLengthEntry = table.getEntry("tvert");
        ledModeEntry = table.getEntry("ledMode");
        camModeEntry = table.getEntry("camMode");
        pipelineEntry = table.getEntry("pipeline");
        getPipelineEntry = table.getEntry("getpipe");
        cameraStreamEntry = table.getEntry("stream");
        snapshotModeEntry = table.getEntry("snapshot");

        timingTestEntry = table.getEntry("TIMING_TEST_ENTRY");
    }

    /**
     * @return if limelight is connected
     */
    public boolean isConnected() {
        // Force an update and get current time
        timingTestEntryValue = !timingTestEntryValue; // flip test value
//        timingTestEntry.forceSetBoolean(timingTestEntryValue);
        timingTestEntry.setBoolean(timingTestEntryValue);
        long currentTime = timingTestEntry.getLastChange();

        // Get most recent update from limelight
        long lastUpdate = latencyEntry.getLastChange();

        // Calculate limelights last update
        long timeDifference = currentTime - lastUpdate;
        boolean connected = timeDifference < MAX_UPDATE_TIME;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Limelight Connected", connected);
            SmartDashboard.putNumber("Limelight Time Difference", timeDifference);
        }

        return connected;
    }

    /**
     * @param targetHeightThreshold Height threshold for target
     * @param minRatio              Min ratio for the blue aspect ratio
     * @param maxRatio              Max ratio for the blue aspect ratio
     * @param angleThreshold        maximum skew the target can have
     * @return Whether or not the limelight has a target in view
     */
    public boolean hasValidTarget(
            double targetHeightThreshold,
            double minRatio, double maxRatio,
            double angleThreshold) {
        return hasAnyTarget()
                & hasValidHeight(targetHeightThreshold)
                & hasValidBlueAspectRatio(minRatio, maxRatio)
                & hasValidBlueOrientation(angleThreshold);
    }

    // Not final in case user wants
    // to change them at runtime
    public double DEFAULT_TARGET_HEIGHT_THRESHOLD = 7;
    public double DEFAULT_MIN_ASPECT_RATIO = 1.2;
    public double DEFAULT_MAX_ASPECT_RATIO = 3.3;
    public double DEFAULT_ANGLE_THRESHOLD = 25;

    /**
     * @return Whether or not the limelight has a target in view
     */
    public boolean hasValidTarget() {
        if(SmartDashboard.getBoolean("CV_FILTER_OVERRIDE", false)) {
            return hasValidTarget(
                    SmartDashboard.getNumber("HEIGHT_THRESHOLD",
                            DEFAULT_TARGET_HEIGHT_THRESHOLD),
                    SmartDashboard.getNumber("MIN_ASPECT_RATIO",
                            DEFAULT_MIN_ASPECT_RATIO),
                    SmartDashboard.getNumber("MAX_ASPECT_RATIO",
                            DEFAULT_MAX_ASPECT_RATIO),
                    SmartDashboard.getNumber("SKEW_THRESHOLD",
                            DEFAULT_ANGLE_THRESHOLD));
        }
        return hasValidTarget(
                DEFAULT_TARGET_HEIGHT_THRESHOLD,
                DEFAULT_MIN_ASPECT_RATIO,
                DEFAULT_MAX_ASPECT_RATIO,
                DEFAULT_ANGLE_THRESHOLD);
    }

    /* Commonly Used Contour Information */
    // Whether the limelight has any valid targets (0 or 1)
    private final NetworkTableEntry validTargetEntry;

    /**
     * Decides if a target shows up on limelight screen
     * @return If it has any target
     */
    public boolean hasAnyTarget() {
        boolean validTarget = validTargetEntry.getDouble(0) > 0.5;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Limelight/Valid Target", validTarget);
        }

        return validTarget;
    }

    /**
     * @param targetHeightThreshold Height threshold for target
     * @return If the target fits the height threshold
     */
    public boolean hasValidHeight(double targetHeightThreshold) {
        boolean validHeight = getTargetYAngle() < targetHeightThreshold;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Limelight/Valid Height", validHeight);
        }

        return validHeight;
    }

    /**
     * The blue aspect ratio is the ratio of the width to height of the rotated
     * rectangle.
     *
     * @param minRatio Min ratio for the blue aspect ratio
     * @param maxRatio Max ratio for the blue aspect ratio
     * @return If the blue aspect ratio fits the thresholds
     */
    public boolean hasValidBlueAspectRatio(double minRatio, double maxRatio) {
        double aspectRatio = getHorizontalSideLength() / getVerticalSideLength();
        boolean validRatio = aspectRatio > minRatio && aspectRatio < maxRatio;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Limelight/Valid Ratio", validRatio);
            SmartDashboard.putNumber("Limelight/Aspect Ratio", aspectRatio);
        }

        return validRatio;
    }

    /**
     * @param angleThreshold maximum skew the target can have
     * @return if the skew is less than the maximum skew
     */
    public boolean hasValidBlueOrientation(double angleThreshold) {
        double skew = Math.abs(getTargetSkew());
        boolean validOrientation = Math.min(skew, 90.0 - skew) <= angleThreshold;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Limelight/Valid Skew", validOrientation);
            SmartDashboard.putNumber("Limelight/Skew Value", Math.min(skew, 90.0 - skew));
        }

        return validOrientation;
    }

    // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    public final double MIN_X_ANGLE = -27;
    public final double MAX_X_ANGLE = 27;
    public final double X_ANGLE_SHIFT = -1.5;
    private final NetworkTableEntry xAngleEntry;

    /**
     * @return Horizontal side length of the target
     */
    public double getTargetXAngle() {

        double X_SHIFT = SmartDashboard.getNumber("Limelight/X_SHIFT", 1000);
        if(X_SHIFT > 694) SmartDashboard.putNumber("Limelight/X_SHIFT", X_ANGLE_SHIFT);
        return xAngleEntry.getDouble(0) + X_SHIFT;
    }

    // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    public final double MIN_Y_ANGLE = -20.5;
    public final double MAX_Y_ANGLE = 20.5;
    private final NetworkTableEntry yAngleEntry;

    /**
     * @return The vertical angle of the target
     */
    public double getTargetYAngle() {
        return yAngleEntry.getDouble(0);
    }

    // Target Area (0% of image to 100% of image)
    public final double MIN_TARGET_AREA = 0;
    public final double MAX_TARGET_AREA = 1;
    private final NetworkTableEntry targetAreaEntry;

    /**
     * @return Percent of the screen the target takes up on a scale of 0 to 1
     */
    public double getTargetArea() {
        // Lime light returns a double from 0 - 100
        // Divide by 100 to scale number from 0 - 1
        return Math.min(targetAreaEntry.getDouble(0) / 100.0, 1);
    }

    // Skew or rotation (-90 degrees to 0 degrees)
    public final double MIN_SKEW = -90;
    public final double MAX_SKEW = 0;
    private final NetworkTableEntry targetSkewEntry;

    /**
     * @return Skew of the Target
     */
    public double getTargetSkew() {
        return targetSkewEntry.getDouble(0);
    }

    // The pipeline’s latency contribution (ms) Add at
    // least 11ms for image capture latency.
    public final double IMAGE_CAPTURE_LATENCY = 11;
    private final NetworkTableEntry latencyEntry;

    /**
     * @return Latency of limelight information in milli-seconds
     */
    public double getLatencyMs() {
        // Add Image Capture Latency to get more accurate result
        return latencyEntry.getDouble(0) + IMAGE_CAPTURE_LATENCY;
    }

    // Pixel information returned from these functions
    public final double MIN_SIDE_LENGTH = 0;
    public final double MAX_SIDE_LENGTH = 320;

    private final NetworkTableEntry shortestSideLengthEntry;

    /**
     * Sidelength of shortest side of the fitted bounding box (0 - 320 pixels)
     *
     * @return Shortest side length of target in pixels
     */
    public double getShortestSideLength() {
        return shortestSideLengthEntry.getDouble(0);
    }

    private final NetworkTableEntry longestSideLengthEntry;

    /**
     * Sidelength of longest side of the fitted bounding box (0 - 320 pixels)
     *
     * @return Longest side length of the target in pixels
     */
    public double getLongestSideLength() {
        return longestSideLengthEntry.getDouble(0);
    }

    private final NetworkTableEntry horizontalSideLengthEntry;

    /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     *
     * @return Horizontal side length of target in pixels
     */
    public double getHorizontalSideLength() {
        return horizontalSideLengthEntry.getDouble(0);
    }

    private final NetworkTableEntry verticalSideLengthEntry;

    /**
     * Vertical sidelength of the rough bounding box (0 - 320 pixels)
     *
     * @return Vertical side length of target in pixels
     */
    public double getVerticalSideLength() {
        return verticalSideLengthEntry.getDouble(0);
    }

    /* Advanced Usage with Raw Contours (Not sent by default) */
    // Raw Contours are formatted as tx0, ty0, tx1, ty1, tx2, ty2
    // So to make this easier, you pass an int and it formats it

    /**
     * @param target Target to read X Angle from
     * @return X Angle of corresponding target
     */
    public double getRawTargetXAngle(int target) {
        return table.getEntry("tx" + target).getDouble(0);
    }

    /**
     * @param target Target to read Y Angle from
     * @return Y Angle of corresponding target
     */
    public double getRawTargetYAngle(int target) {
        return table.getEntry("ty" + target).getDouble(0);
    }

    /**
     * @param target Target to read Area from
     * @return Percent of the screen the corresponding target takes up on a scale of
     *         0 to 1
     */
    public double getRawTargetArea(int target) {
        // Lime light returns a double from 0 - 100
        // Divide by 100 to scale number from 0 - 1
        return Math.min(table.getEntry("ta" + target).getDouble(0) / 100.0, 1);
    }

    /**
     * @param target Target to read Skew from
     * @return Skew of corresponding target
     */
    public double getRawTargetSkew(int target) {
        return table.getEntry("ts" + target).getDouble(0);
    }

    /**
     * @param crosshair Crosshair to read coords from
     * @return X Coordinate of corresponding crosshair
     */
    public double getCustomRawCrosshairX(int crosshair) {
        return table.getEntry("cx" + crosshair).getDouble(0);
    }

    /**
     * @param crosshair Crosshair to read coords from
     * @return Y Coordinate of corresponding crosshair
     */
    public double getRawCrosshairY(int crosshair) {
        return table.getEntry("cy" + crosshair).getDouble(0);
    }

    /* Custom Grip Values */
    // Return data given by custom GRIP pipeline
    /**
     * @param element Name of double provided by GRIP Pipeline
     * @return Double provided by GRIP Pipeline
     */
    public double getCustomDouble(String element) {
        return table.getEntry(element).getDouble(0);
    }

    /**
     * @param element Name of Number to set on Network Table
     * @param value   Value to set the Number on the Network Table to
     * @return Whether or not the write was successful
     */
    public boolean setCustomNumber(String element, Number value) {
        return table.getEntry(element).setNumber(value);
    }

    /**
     * @param element Name of String provided by GRIP Pipeline
     * @return String provided by GRIP Pipeline
     */
    public String getCustomString(String element) {
        return table.getEntry(element).getString("");
    }

    /**
     * @param element Name of String to set on Network Table
     * @param value   Value to set the Sting on the Network Table to
     * @return Whether or not the write was successful
     */
    public boolean setCustomString(String element, String value) {
        return table.getEntry(element).setString(value);
    }

    /* Camera Controls (Use Enums to prevent invalid inputs) */
    // LEDMode
    public enum LEDMode {
        PIPELINE(0), // Use LED mode set in pipeline
        FORCE_OFF(1), // Force LEDs off
        FORCE_BLINK(2), // Force LEDs to blink
        FORCE_ON(3); // Force LEDs on

        private final int value;

        LEDMode(int value) {
            this.value = value;
        }

        public int getCodeValue() {
            return value;
        }

        public static LEDMode fromCodeValue(int value) {
            for(LEDMode mode: LEDMode.values()) {
                if(mode.value == value) {
                    return mode;
                }
            }
            return null; // not found
        }
    }

    private final NetworkTableEntry ledModeEntry;

    /**
     * @param mode Specified LED Mode to set the limelight to
     */
    public void setLEDMode(LEDMode mode) {
        ledModeEntry.setNumber(mode.getCodeValue());
    }

    /**
     * @return the current LED Mode
     */
    public LEDMode getLEDMode() {
        return LEDMode.fromCodeValue(ledModeEntry.getNumber(-1).intValue());
    }

    // CAM_MODE
    public enum CamMode {
        VISION(0), // Use limelight for CV
        DRIVER(1); // Use limelight for driving

        private final int value;

        CamMode(int value) {
            this.value = value;
        }

        public int getCodeValue() {
            return value;
        }

        public static CamMode fromCodeValue(int value) {
            for(CamMode mode: CamMode.values()) {
                if(mode.value == value) {
                    return mode;
                }
            }
            return null; // not found
        }
    }

    private final NetworkTableEntry camModeEntry;

    /**
     * @param mode Specified Cam Mode to set the limelight to
     */
    public void setCamMode(CamMode mode) {
        camModeEntry.setNumber(mode.getCodeValue());
    }

    /**
     * @return the current Cam Mode
     */
    public CamMode getCamMode() {
        return CamMode.fromCodeValue(camModeEntry.getNumber(-1).intValue());
    }

    // PIPELINE
    private final NetworkTableEntry pipelineEntry;

    /**
     * @param pipeline Specified pipeline to set the limelight to
     */
    public void setPipeline(int pipeline) {
        // Prevent input of invalid pipelines
        if (pipeline >= 0 && pipeline <= 9) {
            pipelineEntry.setNumber(pipeline);
        }
    }

    private final NetworkTableEntry getPipelineEntry;

    public double getPipeline() {
        return getPipelineEntry.getDouble(0);
    }

    // STREAM
    public enum CameraStream { // PIP = Picture-In-Picture
        STANDARD(0), // Shows limelight and secondary camera side by side
        PIP_MAIN(1), // Shows the secondary camera along with and within the limelight camera
        PIP_SECONDARY(2); // Shows the limelight along with and within the limelight camera

        private final int value;

        CameraStream(int value) {
            this.value = value;
        }

        public int getCodeValue() {
            return value;
        }
    }

    private final NetworkTableEntry cameraStreamEntry;

    /**
     * @param stream Specified Camera Stream to set the limelight to
     */
    public void setCameraStream(CameraStream stream) {
        cameraStreamEntry.setNumber(stream.getCodeValue());
    }

    // SNAPSHOT_MODE
    public enum SnapshotMode {
        STOP(0), // Don't take snapshots
        TWO_PER_SECOND(1);

        private final int value;

        SnapshotMode(int value) {
            this.value = value;
        }

        public int getCodeValue() {
            return value;
        }
    }

    private final NetworkTableEntry snapshotModeEntry;

    /**
     * @param mode Specified Snapshot Mode to set the limelight to
     */
    public void setSnapshotMode(SnapshotMode mode) {
        snapshotModeEntry.setNumber(mode.getCodeValue());
    }
}