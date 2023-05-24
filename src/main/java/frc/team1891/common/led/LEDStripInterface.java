package frc.team1891.common.led;

/**
 * An interface to handle control over a simple LED strip, or segments of it.
 */
@SuppressWarnings("unused")
public interface LEDStripInterface {
    /**
     * Returns the length of the leds
     */
    int length();

    /**
     * Sends the data currently on the buffer to the LED strip.
     */
    void update();

    /**
     * Sets the hue (HSV) of the pixel at the given index using a default saturation and value.
     * @param index the target pixel
     * @param hue hue
     */
    void setHue(int index, int hue);

    /**
     * Sets the HSV of the pixel at the given index.
     * @param index the target pixel
     * @param hue hue
     * @param sat saturation
     * @param val value
     */
    void setHSV(int index, int hue, int sat, int val);

    /**
     * Sets the RGB of the pixel at the given index.
     * @param index the target pixel
     * @param r red
     * @param g green
     * @param b blue
     */
    void setRGB(int index, int r, int g, int b);

    /**
     * Sets the hue (HSV) of all the pixels using a default saturation and value.
     * @param hue hue
     */
    void setAllHue(int hue);

    /**
     * Sets the HSV of all the pixels.
     * @param hue hue
     * @param sat saturation
     * @param val value
     */
    void setAllHSV(int hue, int sat, int val);

    /**
     * Sets the RGB of all the pixels.
     * @param r red
     * @param g green
     * @param b blue
     */
    void setAllRGB(int r, int g, int b);

    /**
     * Turns all pixels off.
     */
    void off();

    /**
     * Clears the buffer
     */
    default void clear() {
        off();
    }
}
