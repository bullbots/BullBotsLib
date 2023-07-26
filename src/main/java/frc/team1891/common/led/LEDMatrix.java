package frc.team1891.common.led;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

/**
 * A wrapper class to handle control over a grid shaped LED strip.
 */
@SuppressWarnings("unused")
public class LEDMatrix implements LEDMatrixInterface {
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    private final LEDStrip parentStrip;
    private final int startIndex;
    private final int length;
    private final int numRows, numCols;
    private final boolean serpentine;

    /**
     * Creates a new {@link LEDMatrix} to control a rectangular LED panel.
     * @param parentStrip the parent LEDStrip this is a part of
     * @param startIndex the first index of the matrix on the parent strip
     * @param numRows number of rows
     * @param numCols number of columns
     * @param serpentine wiring of LEDs winds back and forth
     */
    public LEDMatrix(LEDStrip parentStrip, int startIndex, int numRows, int numCols, boolean serpentine) {
        this.parentStrip = parentStrip;
        this.startIndex = startIndex;
        this.length = numRows * numCols;
        this.numRows = numRows;
        this.numCols = numCols;
        this.serpentine = serpentine;
    }

    @Override
    public int length() {
        return length;
    }

    @Override
    public int rows() {
        return numRows;
    }

    @Override
    public int cols() {
        return numCols;
    }

    @Override
    public void update() {
        parentStrip.update();
    }

    /**
     * Returns the index of an (x, y) coordinate, accounting for serpentine wiring.
     * @param x positive to the left
     * @param y positive downwards
     * @return the index of the LED as it's wired.
     */
    public int oneDimensionalIndexOf(int x, int y) {
        if (serpentine) {
            return ((y % 2) == 0) ? (y * numCols + x) : ((y + 1) * numCols - 1 - x);
        }
        return (y * numCols + x);
    }

    /**
     * Returns x coordinate from top left corner.
     * @param index one dimensional index, accounting for serpentine wiring
     * @return x coordinate of the index
     */
    public int xOf(int index) {
        if (serpentine && (index / numCols) % 2 == 1) {
            return numCols - (index % numCols);
        }
        return index % numCols;
    }

    /**
     * Returns y coordinate from top left corner (down being positive).
     * @param index one dimensional index, accounting for serpentine wiring
     * @return y coordinate of the index
     */
    public int yOf(int index) {
        return index / numCols;
    }

    @Override
    public void setHue(int x, int y, int hue) {
        this.setHSV(oneDimensionalIndexOf(x, y), hue, 255, 128);
    }

    @Override
    public void setHue(int index, int hue) {
        parentStrip.setHSV(startIndex + index, hue, 255, 128);
    }

    @Override
    public void setHSV(int x, int y, int hue, int sat, int val) {
        this.setHSV(oneDimensionalIndexOf(x, y), hue, sat, val);
    }

    @Override
    public void setHSV(int index, int hue, int sat, int val) {
        parentStrip.setHSV(startIndex + index, hue, sat, val);
    }

    @Override
    public void setRGB(int x, int y, int r, int g, int b) {
        this.setRGB(oneDimensionalIndexOf(x, y), r, g, b);
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        parentStrip.setRGB(startIndex + index, r, g, b);
    }

    @Override
    public boolean setMatrixHSV(Mat matrix) {
        if (checkMatrix(matrix)) {
            for (int i = 0; i < numRows; ++i) {
                for (int j = 0; j < numCols; ++j) {
                    double[] element = matrix.get(i, j);
                    int curBufIndex = startIndex + oneDimensionalIndexOf(j, i);
                    parentStrip.setHSV(curBufIndex, (int) element[0], (int) element[1], (int) element[2]);
                }
            }
            return true;
        }
        return false;
    }

    @Override
    public boolean setMatrixRGB(Mat matrix) {
        if (checkMatrix(matrix)) {
            for (int i = 0; i < numRows; ++i) {
                for (int j = 0; j < numCols; ++j) {
                    double[] element = matrix.get(i, j);
                    int curBufIndex = startIndex + oneDimensionalIndexOf(j, i);
                    parentStrip.setRGB(curBufIndex, (int) element[0], (int) element[1], (int) element[2]);
                }
            }
            return true;
        }
        return false;
    }

    /**
     * Creates a {@link Mat} of proper dimensions according to the matrix provided
     * @param ledMatrix the LED matrix whose dimensions this Mat should match
     * @return a new Mat
     */
    public static Mat createEmptyMatrix(LEDMatrixInterface ledMatrix) {
        return new Mat(ledMatrix.rows(), ledMatrix.cols(), CvType.CV_8UC3, new Scalar(0, 0, 0));
    }

    /**
     * Creates a {@link Mat} with the given dimensions
     * @param rows height
     * @param cols width
     * @return a new Mat
     */
    public static Mat createEmptyMatrix(int rows, int cols) {
        return new Mat(rows, cols, CvType.CV_8UC3, new Scalar(0, 0, 0));
    }

    @Override
    public void off() {
        for (var i = 0; i < length; ++i) {
            parentStrip.setRGB(startIndex + i, 0, 0, 0);
        }
    }
}
