package frc.team1891.common.led;

/**
 * An interface for creating custom patterns for controlling LED strips more easily.
 */
@SuppressWarnings("unused")
public interface LEDStripPattern {
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
     * If an implemented pattern has an end to it this should return true
     * @return true if the pattern is finished
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
     * Creates a basic {@link LEDStripPattern} that sets all the pixels in the LED strip to a given RGB color.
     * @param r red
     * @param g green
     * @param b blue
     * @return the created {@link LEDStripPattern}
     */
    static LEDStripPattern setRGB(int r, int g, int b) {
        return new LEDStripPattern() {
            @Override
            public void draw(LEDStripInterface leds) {
                leds.setAllRGB(r, g, b);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    /**
     * Creates a basic {@link LEDStripPattern} that sets all the pixels in the LED strip to a given hue (HSV with default saturation and value).
     * @param hue hue
     * @return the created {@link LEDStripPattern}.
     */
    static LEDStripPattern setHue(int hue) {
        return new LEDStripPattern() {
            @Override
            public void draw(LEDStripInterface leds) {
                leds.setAllHue(hue);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    /**
     * Creates a basic {@link LEDStripPattern} that sets all the pixels in the LED strip to a given HSV color.
     * @param hue hue
     * @param sat saturation
     * @param val value
     * @return the created {@link LEDStripPattern}.
     */
    static LEDStripPattern setHSV(int hue, int sat, int val) {
        return new LEDStripPattern() {
            @Override
            public void draw(LEDStripInterface leds) {
                leds.setAllHSV(hue, sat, val);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    /**
     * An {@link LEDStripPattern} that alternates between two other {@link LEDStripPattern}.
     */
    class AlternatingPattern implements LEDStripPattern {
        private final double timeInterval;
        private final LEDStripPattern pattern1, pattern2;
        /**
         * Creates a new {@link LEDStripPattern} that alternates between two other {@link LEDStripPattern}.
         * @param timeInterval the time interval between switching patterns
         * @param pattern1 first pattern
         * @param pattern2 second pattern
         */
        public AlternatingPattern(double timeInterval, LEDStripPattern pattern1, LEDStripPattern pattern2) {
            this.timeInterval = timeInterval;
            this.pattern1 = pattern1;
            this.pattern2 = pattern2;
        }

        /**
         * Creates a new {@link LEDStripPattern} that flashes the given pattern on and off.
         * @param timeInterval the time interval between on and off
         * @param pattern pattern
         */
        public AlternatingPattern(double timeInterval, LEDStripPattern pattern) {
            this.timeInterval = timeInterval;
            this.pattern1 = pattern;
            this.pattern2 = LEDStripPatterns.OFF;
        }

        @Override
        public void draw(LEDStripInterface leds) {
            if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
                pattern1.draw(leds);
            } else {
                pattern2.draw(leds);
            }
        }
    }
}
