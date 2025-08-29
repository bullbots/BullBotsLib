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

    default boolean checkValidIndex(int index) {
        return 0 <= index && index < length();
    }

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
     * Sets the hue (HSV) of all the pixels in the given range using a default saturation and value.
     * @param startIndex start pixel
     * @param endIndex end pixel
     * @param hue hue
     */
    default void setRangeHue(int startIndex, int endIndex, int hue) {
        for (var i = startIndex; i < endIndex; i++) {
            setHue(i, hue);
        }
    }

    /**
     * Sets the HSV of all the pixels in the given range.
     * @param startIndex start pixel
     * @param endIndex end pixel
     * @param hue hue
     * @param sat saturation
     * @param val value
     */
    default void setRangeHSV(int startIndex, int endIndex, int hue, int sat, int val) {
        for (var i = startIndex; i < endIndex; i++) {
            setHSV(i, hue, sat, val);
        }
    }

    /**
     * Sets the RGB of all the pixels in the given range.
     * @param startIndex start pixel
     * @param endIndex end pixel
     * @param r red
     * @param g green
     * @param b blue
     */
    default void setRangeRGB(int startIndex, int endIndex, int r, int g, int b) {
        for (var i = startIndex; i < endIndex; i++) {
            setRGB(i, r, g, b);
        }
    }

    /**
     * Sets the hue (HSV) of all the pixels using a default saturation and value.
     * @param hue hue
     */
    default void setAllHue(int hue) {
        setRangeHue(0, length(), hue);
    }

    /**
     * Sets the HSV of all the pixels.
     * @param hue hue
     * @param sat saturation
     * @param val value
     */
    default void setAllHSV(int hue, int sat, int val) {
        setRangeHSV(0, length(), hue, sat, val);
    }

    /**
     * Sets the RGB of all the pixels.
     * @param r red
     * @param g green
     * @param b blue
     */
    default void setAllRGB(int r, int g, int b) {
        setRangeRGB(0, length(), r, g, b);
    }

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

    /**
     * Flashes between two hues on the given time interval.
     * @param index the target pixel
     * @param timeInterval the interval to flash in seconds
     * @param hueA first hue
     * @param hueB second hue
     */
    default void flashHue(int index, double timeInterval, int hueA, int hueB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setHue(index, hueA);
        } else {
            setHue(index, hueB);
        }
    }

    /**
     * Flashes between two HSV colors on the given time interval.
     * @param index the target pixel
     * @param timeInterval the interval to flash in seconds
     * @param hueA first hue
     * @param satA first saturation
     * @param valA first value
     * @param hueB second hue
     * @param satB second saturation
     * @param valB second value
     */
    default void flashHSV(int index, double timeInterval, int hueA, int satA, int valA, int hueB, int satB, int valB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setHSV(index, hueA, satA, valA);
        } else {
            setHSV(index, hueB, satB, valB);
        }
    }

    /**
     * Flashes between two RGB colors on the given time interval.
     * @param index the target pixel
     * @param timeInterval the interval to flash in seconds
     * @param rA first red
     * @param gA first green
     * @param bA first blue
     * @param rB second red
     * @param gB second green
     * @param bB second blue
     */
    default void flashRGB(int index, double timeInterval, int rA, int gA, int bA, int rB, int gB, int bB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setRGB(index, rA, gA, bA);
        } else {
            setRGB(index, rB, gB, bB);
        }
    }

    /**
     * Flashes pixels in the range between two hues on the given time interval.
     * @param startIndex start pixel
     * @param endIndex end pixel
     * @param timeInterval the interval to flash in seconds
     * @param hueA first hue
     * @param hueB second hue
     */
    default void flashRangeHue(int startIndex, int endIndex, double timeInterval, int hueA, int hueB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setRangeHue(startIndex, endIndex, hueA);
        } else {
            setRangeHue(startIndex, endIndex, hueB);
        }
    }

    /**
     * Flashes pixels in the range between two HSV colors on the given time interval.
     * @param startIndex start pixel
     * @param endIndex end pixel
     * @param timeInterval the interval to flash in seconds
     * @param hueA first hue
     * @param satA first saturation
     * @param valA first value
     * @param hueB second hue
     * @param satB second saturation
     * @param valB second value
     */
    default void flashRangeHSV(int startIndex, int endIndex, double timeInterval, int hueA, int satA, int valA, int hueB, int satB, int valB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setRangeHSV(startIndex, endIndex, hueA, satA, valA);
        } else {
            setRangeHSV(startIndex, endIndex, hueB, satB, valB);
        }
    }

    /**
     * Flashes pixels in the range between two RGB colors on the given time interval.
     * @param startIndex start pixel
     * @param endIndex end pixel
     * @param timeInterval the interval to flash in seconds
     * @param rA first red
     * @param gA first green
     * @param bA first blue
     * @param rB second red
     * @param gB second green
     * @param bB second blue
     */
    default void flashRangeRGB(int startIndex, int endIndex, double timeInterval, int rA, int gA, int bA, int rB, int gB, int bB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setRangeRGB(startIndex, endIndex, rA, gA, bA);
        } else {
            setRangeRGB(startIndex, endIndex, rB, gB, bB);
        }
    }

    /**
     * Flashes all pixels between two hues on the given time interval.
     * @param timeInterval the interval to flash in seconds
     * @param hueA first hue
     * @param hueB second hue
     */
    default void flashAllHue(double timeInterval, int hueA, int hueB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setAllHue(hueA);
        } else {
            setAllHue(hueB);
        }
    }

    /**
     * Flashes all pixels between two HSV colors on the given time interval.
     * @param timeInterval the interval to flash in seconds
     * @param hueA first hue
     * @param satA first saturation
     * @param valA first value
     * @param hueB second hue
     * @param satB second saturation
     * @param valB second value
     */
    default void flashAllHSV(double timeInterval, int hueA, int satA, int valA, int hueB, int satB, int valB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setAllHSV(hueA, satA, valA);
        } else {
            setAllHSV(hueB, satB, valB);
        }
    }

    /**
     * Flashes all pixels between two RGB colors on the given time interval.
     * @param timeInterval the interval to flash in seconds
     * @param rA first red
     * @param gA first green
     * @param bA first blue
     * @param rB second red
     * @param gB second green
     * @param bB second blue
     */
    default void flashAllRGB(double timeInterval, int rA, int gA, int bA, int rB, int gB, int bB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setAllRGB(rA, gA, bA);
        } else {
            setAllRGB(rB, gB, bB);
        }
    }
}
