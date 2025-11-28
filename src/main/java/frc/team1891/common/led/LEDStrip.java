package frc.team1891.common.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * A wrapper class to handle control over a simple LED strip.
 */
@SuppressWarnings("unused")
public class LEDStrip implements LEDStripInterface {
    /** The color channel ordering mode for the LED strip. */
    public enum LEDMode {
        /** Red, Green, Blue ordering. */
        RGB,
        /** Red, Blue, Green ordering. */
        RBG,
        /** Green, Red, Blue ordering. */
        GRB,
        /** Green, Blue, Red ordering. */
        GBR,
        /** Blue, Green, Red ordering. */
        BGR,
        /** Blue, Red, Green ordering. */
        BRG
    }

    /** The color channel ordering mode for this LED strip. */
    protected final LEDMode ledMode;

    /** The WPILib LED controller. */
    protected final AddressableLED leds;
    /** The LED data buffer. */
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
        this(port, length, LEDMode.RGB);
    }

    /**
     * Creates a new {@link LEDStrip} with control over an LED strip plugged into the given port.
     * @param port target PWM port
     * @param length number of LEDs
     * @param ledMode mode
     */
    public LEDStrip(int port, int length, LEDMode ledMode) {
        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        leds.setLength(buffer.getLength());
        this.length = length;
        this.ledMode = ledMode;
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
        if (checkValidIndex(index)) {
            setHSV(index, hue, 255, maxValue);
        }
    }

    @Override
    public void setHSV(int index, int hue, int sat, int val) {
        if (checkValidIndex(index)) {
            buffer.setHSV(index, hue, sat, Math.min(val, maxValue));
        }

        if (sat == 0) {
            setRGB(index, val, val, val);
            return;
        }

        // The below algorithm is copied from Color.fromHSV and moved here for
        // performance reasons.

        // Loosely based on
        // https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB
        // The hue range is split into 60 degree regions where in each region there
        // is one rgb component at a low value (m), one at a high value (v) and one
        // that changes (X) from low to high (X+m) or high to low (v-X)

        // Difference between highest and lowest value of any rgb component
        final int chroma = (sat * val) / 255;

        // Because hue is 0-180 rather than 0-360 use 30 not 60
        final int region = (hue / 30) % 6;

        // Remainder converted from 0-30 to 0-255
        final int remainder = (int) Math.round((hue % 30) * (255 / 30.0));

        // Value of the lowest rgb component
        final int m = val - chroma;

        // Goes from 0 to chroma as hue increases
        final int X = (chroma * remainder) >> 8;

        switch (region) {
            case 0:
                setRGB(index, val, X + m, m);
                break;
            case 1:
                setRGB(index, val - X, val, m);
                break;
            case 2:
                setRGB(index, m, val, X + m);
                break;
            case 3:
                setRGB(index, m, val - X, val);
                break;
            case 4:
                setRGB(index, X + m, m, val);
                break;
            default:
                setRGB(index, val, m, val - X);
                break;
        }
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        if (checkValidIndex(index)) {
            int[] rgb = limitRGBBrightness(fromRGBToCustomMode(r, g, b));
            buffer.setRGB(index, rgb[0], rgb[1], rgb[2]);
        }
    }

    @Override
    public void off() {
        for (var i = 0; i < length; ++i) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }

//    private int[] limitRGBBrightness(int r, int g, int b) {
    private int[] limitRGBBrightness(int[] rgb) {
        int sum = rgb[0] + rgb[1] + rgb[2];
        if (sum > maxBrightness) {
            rgb[0] = rgb[0] / (sum / maxBrightness);
            rgb[1] = rgb[1] / (sum / maxBrightness);
            rgb[2] = rgb[2] / (sum / maxBrightness);
        }
        return rgb;
    }

    private int[] fromRGBToCustomMode(int r, int g, int b) {
        return switch (ledMode) {
            case RGB -> new int[] {r, g, b};
            case RBG -> new int[] {r, b, g};
            case GRB -> new int[] {g, r, b};
            case GBR -> new int[] {g, b, r};
            case BGR -> new int[] {b, g, r};
            case BRG -> new int[] {b, r, g};
        };
    }
}
