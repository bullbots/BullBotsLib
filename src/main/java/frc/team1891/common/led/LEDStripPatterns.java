package frc.team1891.common.led;

/**
 * Specific use case patterns free for easy use.
 */
@SuppressWarnings("unused")
public class LEDStripPatterns {
    /** Does nothing. */
    public static final LEDStripPattern NONE = leds -> {};
    /** Turns the LEDs off */
    public static final LEDStripPattern OFF = LEDStripInterface::off;
    /** Animates a simple rainbow moving along the LED strip. */
    public static LEDStripPattern RAINBOW() {
        return new LEDStripPattern() {
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
    }

    /** Flashes between red and bright red. */
    public static LEDStripPattern ERROR() {
        return new LEDStripPattern.AlternatingPattern(.25, LEDStripPattern.setRGB(200, 0, 0), LEDStripPattern.setRGB(150, 0, 0));
    }
    /** Flashes yellow. */
    public static LEDStripPattern WARNING() {
        return new LEDStripPattern.AlternatingPattern(.25, LEDStripPattern.setRGB(160, 160, 50));
    }
}

