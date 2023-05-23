package frc.team1891.common.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.util.function.Consumer;

/**
 * A wrapper class to handle control over a simple LED strip.
 */
@SuppressWarnings("unused")
public class LEDString {
    protected final AddressableLED leds;
    protected final AddressableLEDBuffer buffer;
    public final int length;

    /**
     * Creates a new {@link LEDString} with control over an LED strip plugged into the given port.
     * @param port target PWM port
     * @param length number of LEDs
     */
    public LEDString(int port, int length) {
        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        leds.setLength(buffer.getLength());
        this.length = length;
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
     * Sets the hue (HSV) of the pixel at the given index using a default saturation and value.
     * @param index the target pixel
     * @param hue hue
     */
    public void setHue(int index, int hue) {
        setHue(index, hue, false);
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
        for (var i = 0; i < length; i++) {
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
        for (var i = 0; i < length; i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    /**
     * Turns all pixels off.
     */
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
         * Set the buffer of the LEDString with a pattern.
         * @param leds target {@link LEDString}
         */
        void draw(LEDString leds);

        /**
         * Sets the buffer of the LEDString with a pattern, and updates it to the physical LED strip.
         * @param leds target {@link LEDString}
         */
        default void run(LEDString leds) {
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
         * Consumers can be written like this: (myLEDString) -> {myLEDString.off();}
         * @param consumer consumer
         * @return the created {@link LEDPattern}.
         */
        static LEDPattern fromConsumer(Consumer<LEDString> consumer) {
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
        public void draw(LEDString leds) {
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
        public static final LEDPattern OFF = LEDString::off;
        /** Animates a simple rainbow moving along the LED strip. */
        public static final LEDPattern RAINBOW = new LEDPattern() {
            private int firstHue = 0;
            public void draw(LEDString leds) {

                for (var i = 0; i < leds.length; i++) {
                    // Calculate the hue - hue is easier for rainbows because the color
                    // shape is a circle so only one value needs to precess
                    final var hue = (firstHue + (i * 180 / leds.length)) % 180;
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
}
