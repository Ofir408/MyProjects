
import java.awt.Color;

import biuoop.DrawSurface;
import biuoop.KeyboardSensor;

/**
 * This program is a Paddle class. Paddle is for the user, the tool that with
 * this he can to move and hit the balls in our game.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-05
 */

public class Paddle implements Sprite, Collidable {
    /**
     * used keyboard in order to check input from the keyboard.
     */
    private biuoop.KeyboardSensor keyboard;
    /**
     * the paddle object is represense by a rectangle.
     */
    private Rectangle rec;
    /**
     * the speed of the paddle (just x so double and not velocity object).
     */
    private double dx;
    /**
     * the color of the paddle.
     */
    private Color color;

    /**
     * This is constructor function of Paddle object.
     *
     * @param keyB     - KeyboardSensor object to read from the keyboard.
     * @param rect     - the paddle.
     * @param xSpeed   - dx, each movement move dx to left or to right.
     * @param newColor - the color of the paddle.
     */
    public Paddle(final KeyboardSensor keyB, final Rectangle rect, final double xSpeed, final Color newColor) {
        this.keyboard = keyB;
        this.rec = rect;
        this.dx = xSpeed;
        this.color = newColor;
    }

    /**
     * This is constructor function of Paddle object.
     *
     * @param rect     - the paddle.
     * @param xSpeed   - dx, each movement move dx to left or to right.
     * @param newColor - the color of the paddle.
     */
    public Paddle(final Rectangle rect, final double xSpeed, final Color newColor) {
        this.rec = rect;
        this.dx = xSpeed;
        this.color = newColor;
    }

    /**
     * set this (current) KeyboardSensor.
     *
     * @param k - the new KeyboardSensor.
     */
    public void setKeyBoardSensor(final KeyboardSensor k) {
        this.keyboard = k;
    }

    /**
     * @param c - the new color to set the paddle to.
     */
    public void setColor(final Color c) {
        this.color = c;
    }

    /**
     * @return the color of the current paddle
     */
    public Color getColor() {
        return this.color;
    }

    /**
     * move the paddle right, if possible. Checking with if statements in order
     * do not exit from our screen.
     * @param dt - time passed.
     */
    public void moveRight(double dt) {

        // change to support dt :
        this.dx = this.dx * dt * 60;

        // minus 25 in order prevent going to the end.
        double screenWidth = 800 - 25;
        if (this.rec.getUpperLeft().getX() + this.rec.getWidth() + this.dx < screenWidth) {
            // it means that we can move right

            this.rec = new Rectangle(
                    new Point(this.rec.getUpperLeft().getX() + this.dx, this.rec.getUpperLeft().getY()),
                    this.rec.getWidth(), this.rec.getHeight());
        } else {
            // we can't move right, in order do not exit
            // our screen with the paddle.
            this.rec = new Rectangle(new Point(screenWidth - this.rec.getWidth(), this.rec.getUpperLeft().getY()),
                    this.rec.getWidth(), this.rec.getHeight());
        }
    }

    /**
     * move the paddle left, if possible. Checking with if statements in order
     * do not exit from our screen.
     *
     * @param dt - amount of seconds passed since the last call.
     */
    public void moveLeft(double dt) {
        // plus 25 in order prevent going to the end.
        double currentX = this.rec.getUpperLeft().getX();
        double currentY = this.rec.getUpperLeft().getY();

        // change to support dt :
        this.dx = this.dx * dt * 60;
        if (currentX > 25 + this.dx) {
            // move it left because it's possible to do that.
            this.rec = new Rectangle(new Point(currentX - this.dx, currentY), this.rec.getWidth(),
                    this.rec.getHeight());
        } else {
            // it means we can't move the paddle left without
            // exit the screen
            this.rec = new Rectangle(new Point(25, currentY), this.rec.getWidth(), this.rec.getHeight());
        }

    }

    // Sprite

    /**
     * How does the Paddle move? its timePassed method should check if the
     * "left" or "right" keys are pressed, and if so move it accordingly.
     * @param dt - time passed.
     */
    @Override
    public void timePassed(double dt) {

        if (this.keyboard.isPressed(KeyboardSensor.LEFT_KEY)) {
            this.moveLeft(dt); //
        }
        if (this.keyboard.isPressed(KeyboardSensor.RIGHT_KEY)) {
            this.moveRight(dt); //
        }
    }

    @Override
    /**
     * function to draw the paddle on the screen.
     *
     * @param d
     *            - DrawSurface that helps us the drae the paddle on the screen.
     */
    public void drawOn(final DrawSurface d) {
        d.setColor(this.color);
        d.fillRectangle((int) this.rec.getUpperLeft().getX(), (int) this.rec.getUpperLeft().getY(),
                (int) this.rec.getWidth(), (int) this.rec.getHeight());
    }

    // Collidable
    @Override

    /**
     * @return the "collision shape" of the object.
     */
    public Rectangle getCollisionRectangle() {
        return this.rec;
    }

    @Override
    /**
     * @param collisionPoint
     *            - the intersection point between the paddle and the ball.
     * @param currentVelocity
     *            - the current velocity before the hit.
     * @return new Velocity after the hit, that his calculation depends in which
     *         from the 5 equally-spaced regions it hits.
     */
    public Velocity hit(final Ball ball, final Point collisionPoint, final Velocity currentVelocity) {

        int partNumber;
        double sizeEachPart = this.rec.getWidth() / 5.00;
        double temp = (collisionPoint.getX() - this.rec.getUpperLeft().getX());
        // check if the ball below the paddle, and handle with
        // this situation in order the ball will continue moving.

        if (currentVelocity.getDy() < 0) {
            ball.setCenterPoint(new Point(collisionPoint.getX() + ball.getVelocity().getDx(),
                    collisionPoint.getY() - this.rec.getHeight() + ball.getVelocity().getDy()));
        }
        if (temp <= sizeEachPart) {
            partNumber = 1;
        } else if (temp <= 2 * sizeEachPart) {
            partNumber = 2;
        } else if (temp <= 3 * sizeEachPart) {
            partNumber = 3;
        } else if (temp <= 4 * sizeEachPart) {
            partNumber = 4;
        } else {
            partNumber = 5;
        }

        return Velocity.fromAngleAndSpeed((270 + 30 * partNumber) % 360, currentVelocity.getSpeed());
    }

    @Override
    /**
     * Add this paddle to the game.
     *
     * @param g
     *            - the game to add the current paddle to him.
     */
    public void addToGame(final GameLevel g) {
        g.addCollidable(this);
        g.addSprite(this);
    }

}
