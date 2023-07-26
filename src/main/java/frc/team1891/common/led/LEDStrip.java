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
    public void off() {
        for (var i = 0; i < length; ++i) {
            buffer.setRGB(i, 0, 0, 0);
        }
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
