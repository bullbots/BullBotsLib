package frc.team1891.common.led;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * A wrapper class to handle control over a grid shaped LED strip.
 */
@SuppressWarnings("unused")
public class LEDMatrix implements LEDStripInterface {
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    private final LEDStrip parentStrip;
    private final int startIndex;
    private final int length;
    private final int numRows, numCols;
    private final boolean serpentine;

    /**
     * Creates a new {@link LEDMatrix} with control over an LED strip plugged into the given port.
     * @param parentStrip the parent LEDStrip this is a part of
     * @param startIndex the first index of the matrix on the parent strip
     * @param numRows number of rows
     * @param numCols number of columns
     * @param serpentine wiring of LEDs winds back and forth
     */
    public LEDMatrix(LEDStrip parentStrip, int startIndex, int numRows, int numCols, boolean serpentine) {
        this.parentStrip = parentStrip;
        this.startIndex = startIndex;
        this.length = numRows * numCols;
        this.numRows = numRows;
        this.numCols = numCols;
        this.serpentine = serpentine;
    }

    @Override
    public int length() {
        return length;
    }

    public int rows() {
        return numRows;
    }

    public int cols() {
        return numCols;
    }

    @Override
    public void update() {
        parentStrip.update();
    }

    /**
     * Returns the index of an (x, y) coordinate, accounting for the serpentine wiring.
     * @param x positive to the left
     * @param y positive downwards
     * @return the index of the LED as it's wired.
     */
    public int oneDimensionalIndexOf(int x, int y) {
        if (serpentine) {
            return ((x % 2) == 0) ? startIndex + (x * numCols + y) : startIndex + ((x + 1) * numCols - 1 - y);
        }
        return startIndex + (x * numCols + y);
    }

    /**
     * Sets the hue (HSV) of the pixel at the given index using a default saturation and value.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param hue hue
     */
    public void setHue(int x, int y, int hue) {
        parentStrip.setHSV(oneDimensionalIndexOf(x, y), hue, 255, 128);
    }

    @Override
    public void setHue(int index, int hue) {
        parentStrip.setHSV(startIndex + index, hue, 255, 128);
    }

    /**
     * Sets the HSV of the pixel at the given index.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param hue hue
     * @param sat saturation
     * @param val value
     */
    public void setHSV(int x, int y, int hue, int sat, int val) {
        parentStrip.setHSV(oneDimensionalIndexOf(x, y), hue, sat, val);
    }

    @Override
    public void setHSV(int index, int hue, int sat, int val) {
        parentStrip.setHSV(startIndex + index, hue, sat, val);
    }

    /**
     * Sets the RGB of the pixel at the given index.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param r red
     * @param g green
     * @param b blue
     */
    public void setRGB(int x, int y, int r, int g, int b) {
        parentStrip.setRGB(oneDimensionalIndexOf(x, y), r, g, b);
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        parentStrip.setRGB(startIndex + index, r, g, b);
    }

    @Override
    public void setAllHue(int hue) {
        setAllHSV(hue, 255, 128);
    }

    @Override
    public void setAllHSV(int hue, int sat, int val) {
        for (var i = 0; i < length; i++) {
            parentStrip.setHSV(startIndex + i, hue, sat, val);
        }
    }

    @Override
    public void setAllRGB(int r, int g, int b) {
        for (var i = 0; i < length; i++) {
            parentStrip.setRGB(startIndex + i, r, g, b);
        }
    }

    private boolean checkMatrix(Mat matrix) {
        return matrix.depth() == 0 && matrix.rows() == numRows && matrix.cols() == numCols && matrix.channels() == 3;
    }

    /**
     * Sets the LEDs according to the given matrix
     * @param matrix {@link Mat} of {@link CvType}.CV_8UC3.
     */
    public void setMatrixHSV(Mat matrix) {
        if (checkMatrix(matrix)) {
            for (int i = 0; i < numRows; ++i) {
                for (int j = 0; j < numCols; ++j) {
                    double[] element = matrix.get(i, j);
                    int curBufIndex = oneDimensionalIndexOf(i, j);
                    parentStrip.setHSV(curBufIndex, (int) element[2], (int) element[1], (int) element[0]);
                }
            }
        }
    }

    /**
     * Sets the LEDs according to the given matrix
     * @param matrix {@link Mat} of {@link CvType}.CV_8UC3.
     */
    public void setMatrixRGB(Mat matrix) {
        if (checkMatrix(matrix)) {
            for (int i = 0; i < numRows; ++i) {
                for (int j = 0; j < numCols; ++j) {
                    double[] element = matrix.get(i, j);
                    int curBufIndex = oneDimensionalIndexOf(i, j);
                    parentStrip.setRGB(curBufIndex, (int) element[2], (int) element[1], (int) element[0]);
                }
            }
        }
    }

    @Override
    public void off() {
        for (var i = 0; i < length; ++i) {
            parentStrip.setRGB(startIndex + i, 0, 0, 0);
        }
    }

    /**
     * An interface for creating custom patterns for controlling LED strips more easily.
     */
    public interface LEDPattern {
        /**
         * Set the buffer of the LEDMatrix with a pattern.
         * @param leds target {@link LEDMatrix}
         */
        void draw(LEDMatrix leds);

        /**
         * Sets the buffer of the LEDMatrix with a pattern, and updates it to the physical LED strip.
         * @param leds target {@link LEDMatrix}
         */
        default void run(LEDMatrix leds) {
            draw(leds);
            leds.update();
        }

        /**
         * Creates a basic {@link LEDPattern} that sets all the pixels in the LED strip to a given RGB color.
         * @param r red
         * @param g green
         * @param b blue
         * @return the created {@link LEDPattern}
         */
        static LEDPattern setRGB(int r, int g, int b) {
            return leds -> leds.setAllRGB(r, g, b);
        }

        /**
         * Creates a basic {@link LEDPattern} that sets all the pixels in the LED strip to a given hue (HSV with default saturation and value).
         * @param hue hue
         * @return the created {@link LEDPattern}.
         */
        static LEDPattern setHue(int hue) {
            return leds -> leds.setAllHue(hue);
        }

        /**
         * Creates a basic {@link LEDPattern} that sets all the pixels in the LED strip to a given HSV color.
         * @param hue hue
         * @param sat saturation
         * @param val value
         * @return the created {@link LEDPattern}.
         */
        static LEDPattern setHue(int hue, int sat, int val) {
            return leds -> leds.setAllHSV(hue, sat, val);
        }
    }

    /**
     * An {@link LEDPattern} that alternates between two other {@link LEDPattern}.
     */
    public static class AlternatingPattern implements LEDPattern {
        private final double timeInterval;
        private final LEDPattern pattern1, pattern2;
        /**
         * Creates a new {@link LEDPattern} that alternates between two other {@link LEDPattern}.
         * @param timeInterval the time interval between switching patterns
         * @param pattern1 first pattern
         * @param pattern2 second pattern
         */
        public AlternatingPattern(double timeInterval, LEDPattern pattern1, LEDPattern pattern2) {
            this.timeInterval = timeInterval;
            this.pattern1 = pattern1;
            this.pattern2 = pattern2;
        }

        /**
         * Creates a new {@link LEDPattern} that flashes the given pattern on and off.
         * @param timeInterval the time interval between on and off
         * @param pattern pattern
         */
        public AlternatingPattern(double timeInterval, LEDPattern pattern) {
            this.timeInterval = timeInterval;
            this.pattern1 = pattern;
            this.pattern2 = LEDPatterns.OFF;
        }

        @Override
        public void draw(LEDMatrix leds) {
            long currentTime = System.currentTimeMillis();
            if ((currentTime % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
                pattern1.draw(leds);
            } else {
                pattern2.draw(leds);
            }
        }
    }

    public static class LEDPatterns {
        /** Does nothing. */
        public static final LEDPattern NONE = leds -> {};
        /** Turns the LEDs off */
        public static final LEDPattern OFF = LEDMatrix::off;
        /** Animates a simple rainbow moving diagonally along the LED grid. */
        public static final LEDPattern RAINBOW = new LEDPattern() {
            private int rainbowFirstPixelHue = 0;
            public void draw(LEDMatrix leds) {
                // Diagonal rainbow
                for (int i = 0; i < leds.numRows; i++) {
                    final int rowStartHue = (rainbowFirstPixelHue + (i * 180 / (2 * leds.numRows))) % 180;
                    for (int j = 0; j < leds.numCols; j++) {
                        final int hue = (rowStartHue + (j * 180 / (2 * leds.numCols))) % 180;
                        leds.setHSV(i, j, hue, 255, 128);
                    }
                }
                // Increase by to make the rainbow "move"
                rainbowFirstPixelHue++;
                // Check bounds
                rainbowFirstPixelHue %= 180;
            }
        };

        /** Flashes between red and bright red. */
        public static final LEDPattern ERROR = new AlternatingPattern(.25, LEDPattern.setRGB(200, 0, 0), LEDPattern.setRGB(150, 0, 0));
        /** Flashes yellow. */
        public static final LEDPattern WARNING = new AlternatingPattern(.25, LEDPattern.setRGB(160, 160, 50));
    }
}
