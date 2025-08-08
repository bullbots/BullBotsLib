package frc.team1891.common.led;

/**
 * A wrapper class to handle control only a specific segment of an {@link LEDStrip}.
 */
@SuppressWarnings("unused")
public class LEDStripSegment implements LEDStripInterface {
    public final int startIndex;
    private final int length;
    public final LEDStrip parentStrip;
    protected boolean isReversed = false;

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

    /**
     * Combines two segments to behave as one
     * @param segmentA
     * @param segmentB
     * @return the combined segment
     */
    public static LEDStripSegment combine(LEDStripSegment segmentA, LEDStripSegment segmentB) {
        return new LEDStripSegment(segmentA.parentStrip, segmentA.startIndex, segmentA.length() + segmentB.length()) {
            @Override
            public void setHue(int index, int hue) {
                if (segmentA.checkValidIndex(index)) {
                    segmentA.setHue(index, hue);
                } else {
                    segmentB.setHue(index - segmentA.length(), hue);
                }
            }

            @Override
            public void setHSV(int index, int hue, int sat, int val) {
                if (segmentA.checkValidIndex(index)) {
                    segmentA.setHSV(index, hue, sat, val);
                } else {
                    segmentB.setHSV(index - segmentA.length(), hue, sat, val);
                }
            }

            @Override
            public void setRGB(int index, int r, int g, int b) {
                if (segmentA.checkValidIndex(index)) {
                    segmentA.setRGB(index, r, g, b);
                } else {
                    segmentB.setRGB(index - segmentA.length(), r, g, b);
                }
            }

            @Override
            public void off() {
                segmentA.off();
                segmentB.off();
            }
        };
    }

    /**
     * Reverses the indexes of the LED strip.
     * @param reverse boolean
     */
    public void setReversed(boolean reverse) {
        this.isReversed = reverse;
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
        if (checkValidIndex(index)) {
            parentStrip.setHue(calculateIndex(index), hue);
        }
    }

    @Override
    public void setHSV(int index, int hue, int sat, int val) {
        if (checkValidIndex(index)) {
            parentStrip.setHSV(calculateIndex(index), hue, sat, val);
        }
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        if (checkValidIndex(index)) {
            parentStrip.setRGB(calculateIndex(index), r, g, b);
        }
    }

    @Override
    public void off() {
        for (var i = 0; i < length; ++i) {
            parentStrip.setRGB(startIndex + i, 0, 0, 0);
        }
    }

    /**
     * Returns the correct index according to the parentStrip
     * @param index on this strip
     * @return index on parent strip
     */
    private int calculateIndex(int index) {
        return isReversed ? startIndex + length - 1 - index : startIndex + index;
    }
}
