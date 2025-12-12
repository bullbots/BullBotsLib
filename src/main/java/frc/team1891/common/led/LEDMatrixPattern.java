package frc.team1891.common.led;

import org.opencv.core.Mat;

import java.util.List;

/**
 * An interface for creating custom patterns for controlling LED strips more easily.
 */
@SuppressWarnings("unused")
public interface LEDMatrixPattern {
    /**
     * Set the buffer of the LEDMatrix with a pattern.
     * @param leds target {@link LEDMatrixInterface}
     */
    void draw(LEDMatrixInterface leds);

    /**
     * Sets the buffer of the LEDMatrix with a pattern, and updates it to the physical LED strip.
     * @param leds target {@link LEDMatrixInterface}
     */
    default void run(LEDMatrixInterface leds) {
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
     * Creates a basic {@link LEDMatrixPattern} that sets all the pixels according to a {@link Mat}.
     * @param matrix the RGB matrix to display
     * @return the created {@link LEDMatrixPattern}
     */
    static LEDMatrixPattern setFromRGBMatrix(Mat matrix) {
        return new LEDMatrixPattern() {
            @Override
            public void draw(LEDMatrixInterface leds) {
                leds.setMatrixRGB(matrix);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    /**
     * Creates a basic {@link LEDMatrixPattern} that sets all the pixels according to a {@link Mat}.
     * @param matrix the HSV matrix to display
     * @return the created {@link LEDMatrixPattern}
     */
    static LEDMatrixPattern setFromHSVMatrix(Mat matrix) {
        return new LEDMatrixPattern() {
            @Override
            public void draw(LEDMatrixInterface leds) {
                leds.setMatrixHSV(matrix);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    /**
     * Creates a basic {@link LEDMatrixPattern} that sets all the pixels in the LED strip to a given RGB color.
     * @param r red
     * @param g green
     * @param b blue
     * @return the created {@link LEDMatrixPattern}
     */
    static LEDMatrixPattern setRGB(int r, int g, int b) {
        return new LEDMatrixPattern() {
            @Override
            public void draw(LEDMatrixInterface leds) {
                leds.setAllRGB(r, g, b);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    /**
     * Creates a basic {@link LEDMatrixPattern} that sets all the pixels in the LED strip to a given hue (HSV with default saturation and value).
     * @param hue hue
     * @return the created {@link LEDMatrixPattern}.
     */
    static LEDMatrixPattern setHue(int hue) {
        return new LEDMatrixPattern() {
            @Override
            public void draw(LEDMatrixInterface leds) {
                leds.setAllHue(hue);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    /**
     * Creates a basic {@link LEDMatrixPattern} that sets all the pixels in the LED strip to a given HSV color.
     * @param hue hue
     * @param sat saturation
     * @param val value
     * @return the created {@link LEDMatrixPattern}.
     */
    static LEDMatrixPattern setHue(int hue, int sat, int val) {
        return new LEDMatrixPattern() {
            @Override
            public void draw(LEDMatrixInterface leds) {
                leds.setAllHSV(hue, sat, val);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    /**
     * An {@link LEDMatrixPattern} that alternates between two other {@link LEDMatrixPattern}.
     */
    class AlternatingPattern implements LEDMatrixPattern {
        private final double timeInterval;
        private final LEDMatrixPattern pattern1, pattern2;
        /**
         * Creates a new {@link LEDMatrixPattern} that alternates between two other {@link LEDMatrixPattern}.
         * @param timeInterval the time interval between switching patterns
         * @param pattern1 first pattern
         * @param pattern2 second pattern
         */
        public AlternatingPattern(double timeInterval, LEDMatrixPattern pattern1, LEDMatrixPattern pattern2) {
            this.timeInterval = timeInterval;
            this.pattern1 = pattern1;
            this.pattern2 = pattern2;
        }

        /**
         * Creates a new {@link LEDMatrixPattern} that flashes the given pattern on and off.
         * @param timeInterval the time interval between on and off
         * @param pattern pattern
         */
        public AlternatingPattern(double timeInterval, LEDMatrixPattern pattern) {
            this.timeInterval = timeInterval;
            this.pattern1 = pattern;
            this.pattern2 = LEDMatrixPatterns.OFF;
        }

        @Override
        public void draw(LEDMatrixInterface leds) {
            if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
                pattern1.draw(leds);
            } else {
                pattern2.draw(leds);
            }
        }
    }

    /**
     * An {@link LEDMatrixPattern} plays through a list of {@link Mat} frames at a given FPS.
     */
    class AnimatedLEDMatrixPattern implements LEDMatrixPattern {
        /** Determines how the animation plays. */
        public enum RunType {
            /** Play the animation once and stop. */
            ONCE,
            /** Loop the animation continuously. */
            CONTINUOUS
        }

        private final RunType runType;
        private final int fps;
        private final List<Mat> frames;

        private long startTime;
        private int currentFrame;

        /**
         * Creates a new AnimatedLEDMatrixPattern that runs continuously.
         * @param fps frames per second
         * @param frames the matrices representing each frame in RGB
         */
        public AnimatedLEDMatrixPattern(int fps, List<Mat> frames) {
            this(RunType.CONTINUOUS, fps, frames);
        }

        /**
         * Creates a new AnimatedLEDMatrixPattern.
         * @param runType continuous or run once
         * @param fps frames per second
         * @param frames the matrices representing each frame in RGB
         */
        public AnimatedLEDMatrixPattern(RunType runType, int fps, List<Mat> frames) {
            this.runType = runType;
            this.fps = fps;
            this.frames = frames;
            this.currentFrame = frames.size() + 1; // in order to trigger reset upon first call to draw
        }

        @Override
        public void draw(LEDMatrixInterface leds) {
            leds.setMatrixRGB(frames.get(currentFrame));

            double currentTime = (System.currentTimeMillis() - startTime) / 1000.;
            currentFrame = (int) (currentTime * fps);
        }

        @Override
        public void run(LEDMatrixInterface leds) {
            if (runType.equals(RunType.CONTINUOUS) && isFinished()) {
                reset();
            }

            LEDMatrixPattern.super.run(leds);
        }

        @Override
        public boolean isFinished() {
            return currentFrame >= frames.size();
        }

        @Override
        public void reset() {
            startTime = System.currentTimeMillis();
            currentFrame = 0;
        }
    }
}
