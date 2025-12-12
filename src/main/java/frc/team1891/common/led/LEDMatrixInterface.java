package frc.team1891.common.led;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * An interface to handle control over an LED matrix.
 */
@SuppressWarnings("unused")
public interface LEDMatrixInterface extends LEDStripInterface {
    /**
     * @return number of rows in the matrix
     */
    int rows();

    /**
     * @return number of columns in the matrix
     */
    int cols();

    /**
     * Sets the hue (HSV) of the pixel at the given index using a default saturation and value.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param hue hue
     */
    void setHue(int x, int y, int hue);

    /**
     * Sets the HSV of the pixel at the given index.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param hue hue
     * @param sat saturation
     * @param val value
     */
    void setHSV(int x, int y, int hue, int sat, int val);

    /**
     * Sets the RGB of the pixel at the given index.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param r red
     * @param g green
     * @param b blue
     */
    void setRGB(int x, int y, int r, int g, int b);

    /**
     * Flashes between two hues on the given time interval.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param timeInterval the interval to flash in seconds
     * @param hueA first hue
     * @param hueB second hue
     */
    default void flashHue(int x, int y, double timeInterval, int hueA, int hueB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setHue(x, y, hueA);
        } else {
            setHue(x, y, hueB);
        }
    }

    /**
     * Flashes between two HSV colors on the given time interval.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param timeInterval the interval to flash in seconds
     * @param hueA first hue
     * @param satA first saturation
     * @param valA first value
     * @param hueB second hue
     * @param satB second saturation
     * @param valB second value
     */
    default void flashHSV(int x, int y, double timeInterval, int hueA, int satA, int valA, int hueB, int satB, int valB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setHSV(x, y, hueA, satA, valA);
        } else {
            setHSV(x, y, hueB, satB, valB);
        }
    }

    /**
     * Flashes between two RGB colors on the given time interval.
     * @param x the target pixel x
     * @param y the target pixel y
     * @param timeInterval the interval to flash in seconds
     * @param rA first red
     * @param gA first green
     * @param bA first blue
     * @param rB second red
     * @param gB second green
     * @param bB second blue
     */
    default void flashRGB(int x, int y, double timeInterval, int rA, int gA, int bA, int rB, int gB, int bB) {
        if ((System.currentTimeMillis() % (int) (timeInterval * 2000)) < (int) (timeInterval * 1000)) {
            setRGB(x, y, rA, gA, bA);
        } else {
            setRGB(x, y, rB, gB, bB);
        }
    }

    /**
     * Checks whether matrix is the correct size.
     * @param matrix the matrix to check
     * @return true if the matrix has the correct dimensions and type
     */
    default boolean checkMatrix(Mat matrix) {
        return matrix.depth() == 0 && matrix.rows() == rows() && matrix.cols() == cols() && matrix.channels() == 3;
    }

    /**
     * Checks whether x coordinate is within bounds.
     * @param x the x coordinate to check
     * @return true if the x coordinate is valid
     */
    default boolean checkX(int x) {
        return 0 <= x && x < cols();
    }

    /**
     * Checks whether y coordinate is within bounds.
     * @param y the y coordinate to check
     * @return true if the y coordinate is valid
     */
    default boolean checkY(int y) {
        return 0 <= y && y < rows();
    }

    /**
     * Sets the LEDs according to the given matrix
     * @param matrix {@link Mat} of {@link CvType}.CV_8UC3.
     * @return true if the operation was successful
     */
    boolean setMatrixHSV(Mat matrix);

    /**
     * Sets the LEDs according to the given matrix
     * @param matrix {@link Mat} of {@link CvType}.CV_8UC3.
     * @return true if the operation was successful
     */
    boolean setMatrixRGB(Mat matrix);
}
