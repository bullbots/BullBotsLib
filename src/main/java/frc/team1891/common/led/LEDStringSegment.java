package frc.team1891.common.led;

/**
 * A wrapper class to handle control only a specific segment of an {@link LEDString}.
 */
@SuppressWarnings("unused")
public class LEDStringSegment implements LEDStringInterface {
    private final int startIndex;
    private final int length;
    private final LEDString parentString;

    /**
     * Creates a new {@link LEDStringSegment} with the given location and size.
     * @param startIndex the first index on the parent LEDString
     * @param length the length of the segment
     * @param parentString the parent LEDString this is a part of
     */
    public LEDStringSegment(int startIndex, int length, LEDString parentString) {
        this.startIndex = startIndex;
        this.length = length;
        this.parentString = parentString;
    }

    @Override
    public int length() {
        return length;
    }

    @Override
    public void update() {
        parentString.update();
    }

    @Override
    public void setHue(int index, int hue) {
        parentString.setHue(startIndex + index, hue);
    }

    @Override
    public void setHue(int index, int hue, boolean clearOthers) {
        parentString.setHue(startIndex + index, hue, clearOthers);
    }

    @Override
    public void setHSV(int index, int hue, int sat, int val) {
        parentString.setHSV(startIndex + index, hue, sat, val);
    }

    @Override
    public void setHSV(int index, int hue, int sat, int val, boolean clearOthers) {
        parentString.setHSV(startIndex + index, hue, sat, val, clearOthers);
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        parentString.setRGB(startIndex + index, r, g, b);
    }

    @Override
    public void setRGB(int index, int r, int g, int b, boolean clearOthers) {
        parentString.setRGB(startIndex + index, r, g, b, clearOthers);
    }

    @Override
    public void setAllHue(int hue) {
        for (var i = startIndex; i < length; ++i) {
            parentString.setHue(i, hue);
        }
    }

    @Override
    public void setAllHSV(int hue, int sat, int val) {
        for (var i = startIndex; i < length; ++i) {
            parentString.setHSV(i, hue, sat, val);
        }
    }

    @Override
    public void setAllRGB(int r, int g, int b) {
        for (var i = startIndex; i < length; ++i) {
            parentString.setRGB(i, r, g, b);
        }
    }

    @Override
    public void off() {
        for (var i = startIndex; i < length; ++i) {
            parentString.setRGB(i, 0, 0, 0);
        }
    }
}
