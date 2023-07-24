package frc.team1891.common.led;

import org.opencv.core.Mat;

/**
 * A wrapper class to handle control only a specific part of an {@link LEDMatrix}.
 */
@SuppressWarnings("unused")
public class LEDMatrixSegment implements LEDMatrixInterface {
    private final int startIndex;
    private final int length;
    private final int numCols, numRows;
    private final LEDMatrix parentMatrix;

    private final int startX, startY;

    /**
     * Creates a new {@link LEDMatrixSegment} to control a smaller rectangle within an {@link LEDMatrix}.
     * @param parentMatrix the parent LEDMatrix this is a part of
     * @param startIndex the first index of the segment on the parent LEDMatrix (not the actual LED index, just the
     *                   index relative to the start of the LEDMatrix)
     * @param numCols the width of the subsection
     * @param numRows the height of the subsection
     */
    public LEDMatrixSegment(LEDMatrix parentMatrix, int startIndex, int numCols, int numRows) {
        this.parentMatrix = parentMatrix;
        this.startIndex = startIndex;
        this.length = numCols * numRows;
        this.numCols = numCols;
        this.numRows = numRows;

        this.startX = parentMatrix.xOf(startIndex);
        this.startY = parentMatrix.yOf(startIndex);
    }

    /**
     * Returns the x coordinate on the parent matrix from the given submatrix coordinate.
     * @param x coordinate in the submatrix
     * @return the parent matrix coordinate
     */
    private int getParentX(int x) {
        return startX + x;
    }

    /**
     * Returns the y coordinate on the parent matrix from the given submatrix coordinate.
     * @param y coordinate in the submatrix
     * @return the parent matrix coordinate
     */
    private int getParentY(int y) {
        return startY + y;
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
        parentMatrix.update();
    }

    /**
     * Returns the index of an (x, y) coordinate.
     * @param x positive to the left
     * @param y positive downwards
     * @return the index of the LED wrapping around in 1 dimension.
     */
    public int oneDimensionalIndexOf(int x, int y) {
        return (y * numCols + x);
    }

    /**
     * Returns x coordinate from top left corner.
     * @param index one dimensional index, accounting for serpentine wiring
     * @return x coordinate of the index
     */
    public int xOf(int index) {
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
        if (checkX(x) && checkY(y)) {
            parentMatrix.setHue(getParentX(x), getParentY(y), hue);
        }
    }

    @Override
    public void setHue(int index, int hue) {
        this.setHue(xOf(index), yOf(index), hue);
    }

    @Override
    public void setHSV(int x, int y, int hue, int sat, int val) {
        if (checkX(x) && checkY(y)) {
            parentMatrix.setHSV(getParentX(x), getParentY(y), hue, sat, val);
        }
    }

    @Override
    public void setHSV(int index, int hue, int sat, int val) {
        this.setHSV(xOf(index), yOf(index), hue, sat, val);
    }

    @Override
    public void setRGB(int x, int y, int r, int g, int b) {
        if (checkX(x) && checkY(y)) {
            parentMatrix.setRGB(getParentX(x), getParentY(y), r, g, b);
        }
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        this.setRGB(xOf(index), yOf(index), r, g, b);
    }

    @Override
    public void setRangeHue(int startIndex, int endIndex, int hue) {
        for (int i = startIndex; i < endIndex; i++) {
            this.setHue(i, hue);
        }
    }

    @Override
    public void setRangeHSV(int startIndex, int endIndex, int hue, int sat, int val) {
        for (int i = startIndex; i < endIndex; i++) {
            this.setHSV(i, hue, sat, val);
        }
    }

    @Override
    public void setRangeRGB(int startIndex, int endIndex, int r, int g, int b) {
        for (int i = startIndex; i < endIndex; i++) {
            this.setRGB(i, r, g, b);
        }
    }

    @Override
    public void setMatrixHSV(Mat matrix) {
        if (checkMatrix(matrix)) {
            for (int i = 0; i < numRows; ++i) {
                for (int j = 0; j < numCols; ++j) {
                    double[] element = matrix.get(i, j);
                    int curBufIndex = oneDimensionalIndexOf(i, j);
                    this.setHSV(curBufIndex, (int) element[2], (int) element[1], (int) element[0]);
                }
            }
        }
    }

    @Override
    public void setMatrixRGB(Mat matrix) {
        if (checkMatrix(matrix)) {
            for (int i = 0; i < numRows; ++i) {
                for (int j = 0; j < numCols; ++j) {
                    double[] element = matrix.get(i, j);
                    int curBufIndex = oneDimensionalIndexOf(i, j);
                    this.setRGB(curBufIndex, (int) element[2], (int) element[1], (int) element[0]);
                }
            }
        }
    }

    @Override
    public void off() {
        for (int i = 0; i < length; i++) {
            this.setRGB(i, 0, 0, 0);
        }
    }
}
