package frc.team1891.common.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.util.function.Consumer;

/**
 * A wrapper class to handle control over a grid shaped LED strip.
 */
@SuppressWarnings("unused")
public class LEDMatrix {
    protected final AddressableLED leds;
    protected final AddressableLEDBuffer buffer;
    public final int length;
    public final int numRows, numCols;
    public final boolean serpentine;

    /**
     * Creates a new {@link LEDMatrix} with control over an LED strip plugged into the given port.
     * @param port target PWM port
     * @param numRows number of rows
     * @param numCols number of columns
     */
    public LEDMatrix(int port, int numRows, int numCols, boolean serpentine) {
        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(numRows * numCols);
        leds.setLength(buffer.getLength());
        this.length = buffer.getLength();
        this.numRows = numRows;
        this.numCols = numCols;
        this.serpentine = serpentine;
    }

    /**
     * Starts updating the leds
     */
    public void start() {
        leds.start();
    }

    /**
     * Stops the leds from updating
     */
    public void stop() {
        leds.stop();
    }

    /**
     * Sends the data currently on the buffer to the LED strip.
     */
    public void update() {
        leds.setData(buffer);
    }

    /**
     * Returns the index of an (x, y) coordinate, accounting for the serpentine wiring.
     * @param x positive to the left
     * @param y positive downwards
     * @return the index of the led as it's wired.
     */
    public int oneDimensionalIndexOf(int x, int y) {
        if (serpentine) {
            return ((x % 2) == 0) ? x * numCols + y : (x + 1) * numCols - 1 - y;
        }
        return x * numCols + y;
    }

    /**
     * Sets the hue (HSV) of the pixel at the given index using a default saturation and value.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param hue hue
     */
    public void setHue(int x, int y, int hue) {
        setHue(oneDimensionalIndexOf(x, y), hue, false);
    }

    /**
     * Sets the hue (HSV) of the pixel at the given index using a default saturation and value.
     * @param index the target pixel
     * @param hue hue
     */
    public void setHue(int index, int hue) {
        setHue(index, hue, false);
    }

    /**
     * Sets the hue (HSV) of the pixel at the given index using a default saturation and value.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param hue hue
     * @param clearOthers turn off the other pixels
     */
    public void setHue(int x, int y, int hue, boolean clearOthers) {
        setHSV(oneDimensionalIndexOf(x, y), hue, 255, 128, clearOthers);
    }

    /**
     * Sets the hue (HSV) of the pixel at the given index using a default saturation and value.
     * @param index the target pixel
     * @param hue hue
     * @param clearOthers turn off the other pixels
     */
    public void setHue(int index, int hue, boolean clearOthers) {
        setHSV(index, hue, 255, 128, clearOthers);
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
        setHSV(oneDimensionalIndexOf(x, y), hue, sat, val, false);
    }

    /**
     * Sets the HSV of the pixel at the given index.
     * @param index the target pixel
     * @param hue hue
     * @param sat saturation
     * @param val value
     */
    public void setHSV(int index, int hue, int sat, int val) {
        setHSV(index, hue, sat, val, false);
    }

    /**
     * Sets the HSV of the pixel at the given index.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param hue hue
     * @param sat saturation
     * @param val value
     * @param clearOthers turn off the other pixels
     */
    public void setHSV(int x, int y, int hue, int sat, int val, boolean clearOthers) {
        setHSV(oneDimensionalIndexOf(x, y), hue, sat, val, clearOthers);
    }

    /**
     * Sets the HSV of the pixel at the given index.
     * @param index the target pixel
     * @param hue hue
     * @param sat saturation
     * @param val value
     * @param clearOthers turn off the other pixels
     */
    public void setHSV(int index, int hue, int sat, int val, boolean clearOthers) {
        if (clearOthers) {
            for (int i = 0; i < length; i++) {
                if (i == index) {
                    buffer.setHSV(i, hue, sat, val);
                } else {
                    buffer.setRGB(i, 0, 0, 0);
                }
            }
        } else {
            buffer.setHSV(index, hue, sat, val);
        }
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
        setRGB(oneDimensionalIndexOf(x, y), r, g, b, false);
    }

    /**
     * Sets the RGB of the pixel at the given index.
     * @param index the target pixel
     * @param r red
     * @param g green
     * @param b blue
     */
    public void setRGB(int index, int r, int g, int b) {
        setRGB(index, r, g, b, false);
    }

    /**
     * Sets the RGB of the pixel at the given index.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param r red
     * @param g green
     * @param b blue
     * @param clearOthers turn off the other pixels
     */
    public void setRGB(int x, int y, int r, int g, int b, boolean clearOthers) {
        setRGB(oneDimensionalIndexOf(x, y), r, g, b, clearOthers);
    }

    /**
     * Sets the RGB of the pixel at the given index.
     * @param index the target pixel
     * @param r red
     * @param g green
     * @param b blue
     * @param clearOthers turn off the other pixels
     */
    public void setRGB(int index, int r, int g, int b, boolean clearOthers) {
        if (clearOthers) {
            for (int i = 0; i < length; i++) {
                if (i == index) {
                    buffer.setRGB(i, r, g, b);
                } else {
                    buffer.setRGB(i, 0, 0, 0);
                }
            }
        } else {
            buffer.setRGB(index, r, g, b);
        }
    }

    /**
     * Sets the hue (HSV) of all the pixels using a default saturation and value.
     * @param hue hue
     */
    public void setAllHue(int hue) {
        setAllHSV(hue, 255, 128);
    }

    /**
     * Sets the HSV of all the pixels.
     * @param hue hue
     * @param sat saturation
     * @param val value
     */
    public void setAllHSV(int hue, int sat, int val) {
        // For every pixel
        for (var i = 0; i < length; i++) {
            // Set the value
            buffer.setHSV(i, hue, sat, val);
        }
    }

    /**
     * Sets the RGB of all the pixels.
     * @param r red
     * @param g green
     * @param b blue
     */
    public void setAllRGB(int r, int g, int b) {
        // For every pixel
        for (var i = 0; i < length; i++) {
            // Set the value
            buffer.setRGB(i, r, g, b);
        }
    }

    /**
     * Turns all pixels off.
     */
    public void off() {
        // For every pixel
        for (var i = 0; i < length; ++i) {
            // Set the value
            buffer.setRGB(i, 0, 0, 0);
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

        /**
         * Creates a basic {@link LEDPattern} from a consumer.
         *
         * Consumers can be written like this: (myLEDMatrix) -> {myLEDMatrix.off();}
         * @param consumer consumer
         * @return the created {@link LEDPattern}.
         */
        static LEDPattern fromConsumer(Consumer<LEDMatrix> consumer) {
            return consumer::accept;
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
