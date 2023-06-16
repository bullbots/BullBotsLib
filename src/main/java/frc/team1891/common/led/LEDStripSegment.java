package frc.team1891.common.led;

/**
 * A wrapper class to handle control only a specific segment of an {@link LEDStrip}.
 */
@SuppressWarnings("unused")
public class LEDStripSegment implements LEDStripInterface {
    private final int startIndex;
    private final int length;
    private final LEDStrip parentStrip;

    /**
     * Creates a new {@link LEDStripSegment} with the given location and size.
     * @param parentStrip the parent LEDStrip this is a part of
     * @param startIndex the first index of segment on the parent LEDStrip
     * @param length the length of the segment
     */
    public LEDStripSegment(LEDStrip parentStrip, int startIndex, int length) {
        this.parentStrip = parentStrip;
        this.startIndex = startIndex;
        this.length = length;
    }

    @Override
    public int length() {
        return length;
    }

    @Override
    public void update() {
        parentStrip.update();
    }

    @Override
    public void setHue(int index, int hue) {
        if (indexCheck(startIndex + index)) {
            parentStrip.setHue((startIndex + index), hue);
        }
    }

    @Override
    public void setHSV(int index, int hue, int sat, int val) {
        if (indexCheck(startIndex + index)) {
            parentStrip.setHSV(startIndex + index, hue, sat, val);
        }
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        if (indexCheck(startIndex + index)) {
            parentStrip.setRGB(startIndex + index, r, g, b);
        }
    }

    @Override
    public void setAllHue(int hue) {
        for (var i = 0; i < length; ++i) {
            parentStrip.setHue(startIndex + i, hue);
        }
    }

    @Override
    public void setAllHSV(int hue, int sat, int val) {
        for (var i = 0; i < length; ++i) {
            parentStrip.setHSV(startIndex + i, hue, sat, val);
        }
    }

    @Override
    public void setAllRGB(int r, int g, int b) {
        for (var i = 0; i < length; ++i) {
            parentStrip.setRGB(startIndex + i, r, g, b);
        }
    }

    @Override
    public void off() {
        for (var i = 0; i < length; ++i) {
            parentStrip.setRGB(startIndex + i, 0, 0, 0);
        }
    }
}
