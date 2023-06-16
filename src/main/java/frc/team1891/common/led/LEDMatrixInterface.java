package frc.team1891.common.led;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

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
     * Checks whether matrix is the correct size.
     */
    default boolean checkMatrix(Mat matrix) {
        return matrix.depth() == 0 && matrix.rows() == rows() && matrix.cols() == cols() && matrix.channels() == 3;
    }

    /**
     * Checks whether x coordinate is within bounds.
     */
    default boolean checkX(int x) {
        return 0 <= x && x < cols();
    }

    /**
     * Checks whether y coordinate is within bounds.
     */
    default boolean checkY(int y) {
        return 0 <= y && y < rows();
    }

    /**
     * Sets the LEDs according to the given matrix
     * @param matrix {@link Mat} of {@link CvType}.CV_8UC3.
     */
    void setMatrixHSV(Mat matrix);

    /**
     * Sets the LEDs according to the given matrix
     * @param matrix {@link Mat} of {@link CvType}.CV_8UC3.
     */
    void setMatrixRGB(Mat matrix);
}
