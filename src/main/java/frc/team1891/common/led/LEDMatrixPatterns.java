package frc.team1891.common.led;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.BooleanSupplier;

/**
 * Specific use case patterns free for easy use.
 */
@SuppressWarnings("unused")
public class LEDMatrixPatterns {
    /**
     * The Snake game built into an LEDMatrixPattern.
     *
     * <p>Any errors or weird code can be blamed on ChatGPT :)</p>
     */
    public static class SnakeGamePattern implements LEDMatrixPattern {

        private final BooleanSupplier up, down, left, right;

        protected int updateRate;

        private SnakeGame game;

        public SnakeGamePattern(BooleanSupplier up, BooleanSupplier down, BooleanSupplier left, BooleanSupplier right) {
            this(7, up, down, left, right);
        }

        public SnakeGamePattern(int updateRate, BooleanSupplier up, BooleanSupplier down, BooleanSupplier left, BooleanSupplier right) {
            this.updateRate = updateRate;

            this.up = up;
            this.down = down;
            this.left = left;
            this.right = right;
        }

        private int i = 0;

        @Override
        public void draw(LEDMatrixInterface leds) {
            // initialize game at proper size
            if (this.game == null) {
                this.game = new SnakeGame(new Board(leds.cols(), leds.rows()));
            }

            if (isFinished()) {
                leds.setRangeRGB(0, game.getHighScore(), 0, 255, 0);
                reset();
                return;
            }

            // read player input
            // TODO: Direction queue
            if (up.getAsBoolean()) {
                game.setDirection(Direction.UP);;
            } else if (down.getAsBoolean()) {
                game.setDirection(Direction.DOWN);
            } else if (left.getAsBoolean()) {
                game.setDirection(Direction.LEFT);
            } else if (right.getAsBoolean()) {
                game.setDirection(Direction.RIGHT);
            }

            // update
            if (i == updateRate) {
                game.update();
                i = 0;

                // draw to the screen
                leds.setMatrixRGB(game.getMatrix());
            }
            i++;
        }

        @Override
        public boolean isFinished() {
            return game.hasLost();
        }

        @Override
        public void reset() {
            System.out.println("Current High Score: " + game.getHighScore());
            System.out.println("Last Score: " + game.getScore());
            game.reset();
        }

        private static class SnakeGame {
            private final Board board;
            private Snake snake;
            private Point fruit;
            private int score;
            private int highScore;

            private boolean hasLost;

            public SnakeGame(Board board) {
                this.board = board;
                reset();
                highScore = 0;
            }

            public void reset() {
                hasLost = false;
                snake = new Snake(new Point(0, board.height/2));
                this.eatFruit();
                score = 0;
            }

            public void eatFruit() {
                this.snake.grow();

                // Generate a new random location for the fruit
                Random rand = new Random();
                Point newFruit;
                do {
                    newFruit = new Point(rand.nextInt(this.board.getWidth()), rand.nextInt(this.board.getHeight()));
                } while (this.snake.getBody().contains(newFruit)); // Make sure the fruit is not on the snake's body

                this.fruit = newFruit;
                score++;
            }

            public int getScore() {return score;}

            public int getHighScore() {return highScore;}

            public Mat getMatrix() {
                Mat mat = board.getMatrix(snake);
                mat.put(fruit.y, fruit.x, 255,0,0);
                return mat;
            }

            public void update() {
                this.snake.move();

                if (this.snake.getHead().equals(this.fruit)) {
                    this.eatFruit();
                }

                hasLost = checkLoss();
            }

            private boolean checkLoss() {
                Point head = this.snake.getHead();

                // Check if the snake's head is out of bounds
                if (!this.board.contains(head)) {
                    saveHighScore();
                    return true;
                }

                // Check if the snake's head collided with its body
                List<Point> body = this.snake.getBody();
                if (body.size() > 1) {
                    for (int i = 1; i < body.size(); i++) {
                        if (head.equals(body.get(i))) {
                            saveHighScore();
                            return true;
                        }
                    }
                }

                return false;
            }

            public boolean hasLost() {
                return hasLost;
            }

            public void setDirection(Direction direction) {
                this.snake.setDirection(direction);
            }

            private void saveHighScore() {
                if (score > highScore) {
                    highScore = score;
                }
            }
        }

        public enum Direction {
            UP(0, -1),
            DOWN(0, 1),
            LEFT(-1, 0),
            RIGHT(1, 0);

            private final int x;
            private final int y;

            Direction(int x, int y) {
                this.x = x;
                this.y = y;
            }

            public int getX() {
                return this.x;
            }

            public int getY() {
                return this.y;
            }
        }

        public static class Snake {
            private Direction direction;
            private final List<Point> body;
            private int length;

            public Snake(Point startPoint) {
                this.direction = Direction.RIGHT;
                this.body = new ArrayList<>();
                this.body.add(startPoint);
                this.length = 1;
            }

            public void setDirection(Direction direction) {
                // prevent moving back on itself
                if (this.direction.getX() + direction.getX() == 0 &&
                        this.direction.getY() + direction.getY() == 0) {
                    return;
                }
                this.direction = direction;
            }

            public Point getHead() {
                return this.body.get(0);
            }

            public List<Point> getBody() {
                return this.body;
            }

            public int getLength() {
                return this.length;
            }

            public void move() {
                Point head = this.getHead();
                Point newHead = new Point(head.x + this.direction.getX(), head.y + this.direction.getY());
                this.body.add(0, newHead);
                if (this.body.size() > this.length) {
                    this.body.remove(this.body.size() - 1);
                }
            }

            public void grow() {
                this.length++;
            }

            public boolean hasCollidedWithBody() {
                Point head = this.getHead();
                for (int i = 1; i < this.body.size(); i++) {
                    if (head.equals(this.body.get(i))) {
                        return true;
                    }
                }
                return false;
            }
        }

        private static class Board {
            private final int width;
            private final int height;

            public Board(int width, int height) {
                this.width = width;
                this.height = height;
            }

            public int getWidth() {
                return this.width;
            }

            public int getHeight() {
                return this.height;
            }

            public boolean contains(Point point) {
                return point.x >= 0 && point.x < this.width && point.y >= 0 && point.y < this.height;
            }

            public Mat getMatrix(Snake snake) {
                Mat matrix = new Mat(height, width, CvType.CV_8UC3);
                for (int i = 0; i < height; i++) {
                    for (int j = 0; j < width; j++) {
                        matrix.put(i, j, 0,0,0);
                    }
                }
                for (int i = 0; i < snake.getLength()-1; i++) {
                    Point body = snake.getBody().get(i);
                    matrix.put(body.y, body.x, 0, 255, Math.min(255, 3*i));
                }
                return matrix;
            }
        }

        public static class Point {
            public int x;
            public int y;

            public Point(int x, int y) {
                this.x = x;
                this.y = y;
            }

            @Override
            public boolean equals(Object obj) {
                if (this == obj) {
                    return true;
                }
                if (!(obj instanceof Point other)) {
                    return false;
                }
                return this.x == other.x && this.y == other.y;
            }

            @Override
            public int hashCode() {
                final int prime = 31;
                int result = 1;
                result = prime * result + this.x;
                result = prime * result + this.y;
                return result;
            }
        }
    }

    /** Does nothing. */
    public static final LEDMatrixPattern NONE = leds -> {};
    /** Turns the LEDs off */
    public static final LEDMatrixPattern OFF = LEDMatrixInterface::off;
    /** Animates a simple rainbow moving diagonally along the LED grid. */
    public static LEDMatrixPattern RAINBOW() {
        return new LEDMatrixPattern() {
            private int rainbowFirstPixelHue = 0;
            public void draw(LEDMatrixInterface leds) {
                // Diagonal rainbow
                for (int i = 0; i < leds.rows(); i++) {
                    final int rowStartHue = (rainbowFirstPixelHue + (i * 180 / (leds.rows() + leds.cols()))) % 180;
                    for (int j = 0; j < leds.cols(); j++) {
                        final int hue = (rowStartHue + (j * 180 / (2 * leds.cols()))) % 180;
                        leds.setHSV(i, j, hue, 255, 128);
                    }
                }
                // Increase by to make the rainbow "move"
                rainbowFirstPixelHue++;
                // Check bounds
                rainbowFirstPixelHue %= 180;
            }
        };
    }

    /** Flashes between red and bright red. */
    public static LEDMatrixPattern ERROR() {
        return new LEDMatrixPattern.AlternatingPattern(.25, LEDMatrixPattern.setRGB(200, 0, 0), LEDMatrixPattern.setRGB(150, 0, 0));
    }
    /** Flashes yellow. */
    public static LEDMatrixPattern WARNING() {
        return new LEDMatrixPattern.AlternatingPattern(.25, LEDMatrixPattern.setRGB(160, 160, 50));
    }
}
