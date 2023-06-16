package frc.team1891.common.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * A wrapper class to handle control over a simple LED strip.
 */
@SuppressWarnings("unused")
public class LEDStrip implements LEDStripInterface {
    protected final AddressableLED leds;
    protected final AddressableLEDBuffer buffer;
    private final int length;
    private int maxBrightness = (255 * 3) / 2;
    private int maxValue = 255;

    /**
     * Creates a new {@link LEDStrip} with control over an LED strip plugged into the given port.
     * @param port target PWM port
     * @param length number of LEDs
     */
    public LEDStrip(int port, int length) {
        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        leds.setLength(buffer.getLength());
        this.length = length;
    }

    @Override
    public int length() {
        return length;
    }

    /**
     * Limit the brightness of the LED strip.
     *
     * Note, this limits the brightness when it takes the color input.  Any colors set before setting the brightness
     * won't change.
     *
     * @param brightness [0,765] the max sum brightness the R, G, and B channels can reach
     */
    public void setMaxBrightness(int brightness) {
        maxBrightness = brightness;
        maxValue = (int) (255 * (brightness / (double) (255 * 3)));
    }

    /**
     * Starts sending data to the leds.
     */
    public void start() {
        leds.start();
    }

    /**
     * Stops the leds from updating.
     */
    public void stop() {
        leds.stop();
    }

    @Override
    public void update() {
        leds.setData(buffer);
    }

    @Override
    public void setHue(int index, int hue) {
        if (indexCheck(index)) {
            setHSV(index, hue, 255, maxValue);
        }
    }

    @Override
    public void setHSV(int index, int hue, int sat, int val) {
        if (indexCheck(index)) {
            buffer.setHSV(index, hue, sat, Math.min(val, maxValue));
        }
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        if (indexCheck(index)) {
            int[] rgb = limitRGBBrightness(r, g, b);
            buffer.setRGB(index, rgb[0], rgb[1], rgb[2]);
        }
    }

    @Override
    public void setAllHue(int hue) {
        setAllHSV(hue, 255, maxValue);
    }

    @Override
    public void setAllHSV(int hue, int sat, int val) {
        val = Math.min(val, maxValue);
        for (var i = 0; i < length; i++) {
            buffer.setHSV(i, hue, sat, val);
        }
    }

    @Override
    public void setAllRGB(int r, int g, int b) {
        int[] rgb = limitRGBBrightness(r, g, b);
        for (var i = 0; i < length; i++) {
            buffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
        }
    }

    @Override
    public void off() {
        for (var i = 0; i < length; ++i) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }

    /**
     * An interface for creating custom patterns for controlling LED strips more easily.
     */
    public interface LEDPattern {
        /**
         * Set the buffer of the LEDStrip with a pattern.
         * @param leds target {@link LEDStripInterface}
         */
        void draw(LEDStripInterface leds);

        /**
         * Sets the buffer of the LEDStrip with a pattern, and updates it to the physical LED strip.
         * @param leds target {@link LEDStripInterface}
         */
        default void run(LEDStripInterface leds) {
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
        static LEDPattern setHSV(int hue, int sat, int val) {
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
        public void draw(LEDStripInterface leds) {
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
        public static final LEDPattern OFF = LEDStripInterface::off;
        /** Animates a simple rainbow moving along the LED strip. */
        public static final LEDPattern RAINBOW = new LEDPattern() {
            private int firstHue = 0;
            public void draw(LEDStripInterface leds) {
                for (var i = 0; i < leds.length(); i++) {
                    // Calculate the hue - hue is easier for rainbows because the color
                    // shape is a circle so only one value needs to precess
                    final var hue = (firstHue + (i * 180 / leds.length())) % 180;
                    // Set the value
                    leds.setHue(i, hue);
                }
                // Increase to make the rainbow "move"
                firstHue ++;
                // Check bounds
                firstHue %= 180;
            }
        };

        /** Flashes between red and bright red. */
        public static final LEDPattern ERROR = new AlternatingPattern(.25, LEDPattern.setRGB(200, 0, 0), LEDPattern.setRGB(150, 0, 0));
        /** Flashes yellow. */
        public static final LEDPattern WARNING = new AlternatingPattern(.25, LEDPattern.setRGB(160, 160, 50));
    }

    private int[] limitRGBBrightness(int r, int g, int b) {
        int[] rgb = new int[] {r, g, b};
        if (r + g + b > maxBrightness) {
            rgb[0] = r / ((r + g + b) / maxBrightness);
            rgb[1] = g / ((r + g + b) / maxBrightness);
            rgb[2] = b / ((r + g + b) / maxBrightness);
        }
        return rgb;
    }
}
