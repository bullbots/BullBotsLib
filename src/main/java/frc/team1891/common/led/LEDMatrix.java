package frc.team1891.common.led;

import org.opencv.core.Core;
import org.opencv.core.Mat;

/**
 * A wrapper class to handle control over a grid shaped LED strip.
 */
@SuppressWarnings("unused")
public class LEDMatrix implements LEDMatrixInterface {
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    private final LEDStrip parentStrip;
    private final int startIndex;
    private final int length;
    private final int numRows, numCols;
    private final boolean serpentine;

    /**
     * Creates a new {@link LEDMatrix} to control a rectangular LED panel.
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

    @Override
    public int rows() {
        return numRows;
    }

    @Override
    public int cols() {
        return numCols;
    }

    @Override
    public void update() {
        parentStrip.update();
    }

    /**
     * Returns the index of an (x, y) coordinate, accounting for serpentine wiring.
     * @param x positive to the left
     * @param y positive downwards
     * @return the index of the LED as it's wired.
     */
    public int oneDimensionalIndexOf(int x, int y) {
        if (serpentine) {
            return ((y % 2) == 0) ? startIndex + (y * numCols + x) : startIndex + ((y + 1) * numCols - 1 - x);
        }
        return (y * numCols + x);
    }

    /**
     * Returns x coordinate from top left corner.
     * @param index one dimensional index, accounting for serpentine wiring
     * @return x coordinate of the index
     */
    public int xOf(int index) {
        if (serpentine && (index / numCols) % 2 == 1) {
            return numCols - (index % numCols);
        }
        return index % numCols;
    }

    /**
     * Returns y coordinate from top left corner (down being positive).
     * @param index one dimensional index, accounting for serpentine wiring
     * @return y coordinate of the index
     */
    public int yOf(int index) {
        return index / numCols;
    }

    @Override
    public void setHue(int x, int y, int hue) {
        this.setHSV(oneDimensionalIndexOf(x, y), hue, 255, 128);
    }

    @Override
    public void setHue(int index, int hue) {
        parentStrip.setHSV(startIndex + index, hue, 255, 128);
    }

    @Override
    public void setHSV(int x, int y, int hue, int sat, int val) {
        this.setHSV(oneDimensionalIndexOf(x, y), hue, sat, val);
    }

    @Override
    public void setHSV(int index, int hue, int sat, int val) {
        parentStrip.setHSV(startIndex + index, hue, sat, val);
    }

    @Override
    public void setRGB(int x, int y, int r, int g, int b) {
        this.setRGB(oneDimensionalIndexOf(x, y), r, g, b);
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        parentStrip.setRGB(startIndex + index, r, g, b);
    }

    @Override
    public void setMatrixHSV(Mat matrix) {
        if (checkMatrix(matrix)) {
            for (int i = 0; i < numRows; ++i) {
                for (int j = 0; j < numCols; ++j) {
                    double[] element = matrix.get(i, j);
                    int curBufIndex = startIndex + oneDimensionalIndexOf(i, j);
                    parentStrip.setHSV(curBufIndex, (int) element[2], (int) element[1], (int) element[0]);
                }
            }
        }
    }

    @Override
    public void setMatrixRGB(Mat matrix) {
        if (checkMatrix(matrix)) {
            for (int i = 0; i < numRows; ++i) {
                for (int j = 0; j < numCols; ++j) {
                    double[] element = matrix.get(i, j);
                    int curBufIndex = startIndex + oneDimensionalIndexOf(i, j);
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
    public interface LEDMatrixPattern {
        /**
         * Set the buffer of the LEDMatrix with a pattern.
         * @param leds target {@link LEDMatrixInterface}
         */
        void draw(LEDMatrixInterface leds);

        /**
         * Sets the buffer of the LEDMatrix with a pattern, and updates it to the physical LED strip.
         * @param leds target {@link LEDMatrixInterface}
         */
        default void run(LEDMatrixInterface leds) {
            draw(leds);
            leds.update();
        }

        /**
         * If an implemented pattern has an end to it this should return true
         */
        default boolean isFinished() {
            return false;
        }

        /**
         * If an implemented pattern doesn't loop, or has a specific starting state, this should revert the pattern to
         * that state.
         */
        default void reset() {}

        /**
         * Creates a basic {@link LEDMatrixPattern} that sets all the pixels in the LED strip to a given RGB color.
         * @param r red
         * @param g green
         * @param b blue
         * @return the created {@link LEDMatrixPattern}
         */
        static LEDMatrixPattern setRGB(int r, int g, int b) {
            return new LEDMatrixPattern() {
                @Override
                public void draw(LEDMatrixInterface leds) {
                    leds.setAllRGB(r, g, b);
                }

                @Override
                public boolean isFinished() {
                    return true;
                }
            };
        }

        /**
         * Creates a basic {@link LEDMatrixPattern} that sets all the pixels in the LED strip to a given hue (HSV with default saturation and value).
         * @param hue hue
         * @return the created {@link LEDMatrixPattern}.
         */
        static LEDMatrixPattern setHue(int hue) {
            return new LEDMatrixPattern() {
                @Override
                public void draw(LEDMatrixInterface leds) {
                    leds.setAllHue(hue);
                }

                @Override
                public boolean isFinished() {
                    return true;
                }
            };
        }

        /**
         * Creates a basic {@link LEDMatrixPattern} that sets all the pixels in the LED strip to a given HSV color.
         * @param hue hue
         * @param sat saturation
         * @param val value
         * @return the created {@link LEDMatrixPattern}.
         */
        static LEDMatrixPattern setHue(int hue, int sat, int val) {
            return new LEDMatrixPattern() {
                @Override
                public void draw(LEDMatrixInterface leds) {
                    leds.setAllHSV(hue, sat, val);
                }

                @Override
                public boolean isFinished() {
                    return true;
                }
            };
        }
    }

    /**
     * An {@link LEDMatrixPattern} that alternates between two other {@link LEDMatrixPattern}.
     */
    public static class AlternatingPattern implements LEDMatrixPattern {
        private final double timeInterval;
        private final LEDMatrixPattern pattern1, pattern2;
        /**
         * Creates a new {@link LEDMatrixPattern} that alternates between two other {@link LEDMatrixPattern}.
         * @param timeInterval the time interval between switching patterns
         * @param pattern1 first pattern
         * @param pattern2 second pattern
         */
        public AlternatingPattern(double timeInterval, LEDMatrixPattern pattern1, LEDMatrixPattern pattern2) {
            this.timeInterval = timeInterval;
            this.pattern1 = pattern1;
            this.pattern2 = pattern2;
        }

        /**
         * Creates a new {@link LEDMatrixPattern} that flashes the given pattern on and off.
         * @param timeInterval the time interval between on and off
         * @param pattern pattern
         */
        public AlternatingPattern(double timeInterval, LEDMatrixPattern pattern) {
            this.timeInterval = timeInterval;
            this.pattern1 = pattern;
            this.pattern2 = LEDMatrixPatterns.OFF;
        }

        @Override
        public void draw(LEDMatrixInterface leds) {
            if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
                pattern1.draw(leds);
            } else {
                pattern2.draw(leds);
            }
        }
    }

    public static class LEDMatrixPatterns {
        /** Does nothing. */
        public static final LEDMatrixPattern NONE = leds -> {};
        /** Turns the LEDs off */
        public static final LEDMatrixPattern OFF = LEDMatrixInterface::off;
        /** Animates a simple rainbow moving diagonally along the LED grid. */
        public static LEDMatrixPattern RAINBOW() {
            return new LEDMatrixPattern() {
                private int rainbowFirstPixelHue = 0;
                public void draw(LEDMatrixInterface leds) {
                    // Diagonal rainbow
                    for (int i = 0; i < leds.rows(); i++) {
                        final int rowStartHue = (rainbowFirstPixelHue + (i * 180 / (leds.rows() + leds.cols()))) % 180;
                        for (int j = 0; j < leds.cols(); j++) {
                            final int hue = (rowStartHue + (j * 180 / (2 * leds.cols()))) % 180;
                            leds.setHSV(i, j, hue, 255, 128);
                        }
                    }
                    // Increase by to make the rainbow "move"
                    rainbowFirstPixelHue++;
                    // Check bounds
                    rainbowFirstPixelHue %= 180;
                }
            };
        }

        /** Flashes between red and bright red. */
        public static LEDMatrixPattern ERROR() {
            return new AlternatingPattern(.25, LEDMatrixPattern.setRGB(200, 0, 0), LEDMatrixPattern.setRGB(150, 0, 0));
        }
        /** Flashes yellow. */
        public static LEDMatrixPattern WARNING() {
            return new AlternatingPattern(.25, LEDMatrixPattern.setRGB(160, 160, 50));
        }
    }
}
