//package secondass;

import java.awt.Color;

import biuoop.DrawSurface;

/**
 * This program is a Ball object.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-04-03
 */
public class Ball {

    /**
     * the point of the ball on the screen.
     */
    private Point centerPoint;
    /**
     * the radius of the ball.
     */
    private int radius;
    /**
     * the Color of the ball.
     */
    private java.awt.Color color;
    /**
     * the Velocity of the ball.
     */
    private Velocity ballVelocity;
    /**
     * GameEnvironment object with all the blocks.
     */
   // private GameEnvironment gameEnvironment;

    // functions:

    /**
     * This is constructor function of Ball object.
     * The Input:
     *
     * @param center    - the place of the ball - it's a point object.
     * @param newRadius - the radius of the ball - an integer number.
     * @param newColor  - the color of the ball.
     *                  The Function Operation: The function initializes
     *                  two input values in the new Ball object.
     */
    public Ball(final Point center, final int newRadius,
                final java.awt.Color newColor) {
        this.centerPoint = center;
        this.radius = newRadius;
        this.color = newColor;
        this.ballVelocity = new Velocity(0.0, 0.0);
    }

    /**
     * This is another constructor function of Ball object.
     * The Input:
     *
     * @param xValue    - the x value of the ball - it's an integer.
     * @param yValue    -  the y value of the ball - it's an integer.
     * @param newRadius - the radius of the ball - an integer number.
     * @param newColor  - the color of the ball.
     *                  The Function Operation: The function create new point
     *                  with the input and then initializes
     *                  two input values in the new Ball object.
     */

    public Ball(final int xValue, final int yValue, final int newRadius,
                final Color newColor) {
        this.centerPoint = new Point(xValue, yValue);
        this.radius = newRadius;
        this.color = newColor;
        this.ballVelocity = new Velocity(0.0, 0.0);
    }


    // Accessory functions.

    /**
     * Return x value of the center point of the ball.
     * The Input: no input, casting to integers because
     * the function in point return double and here we return int.
     *
     * @return the x value of the center point of the ball.
     */
    public final int getX() {
        return (int) this.centerPoint.getX();
    }

    /**
     * Return y value of the center point of the ball.
     * The Input: no input, casting to integers because
     * the function in point return double and here we return int.
     *
     * @return the y value of the center point of the ball.
     */
    public int getY() {
        return (int) this.centerPoint.getY();
    }

    /**
     * Return radius value of the ball.
     * The Input: no input.
     *
     * @return the radius value of the ball.
     */
    public int getSize() {
        return this.radius;
    }

    /**
     * Function name: setRadious
     * set the radius value of the ball.
     * The Input:
     *
     * @param newRadius - the new radius.
     */
    public void setRadius(final int newRadius) {
        this.radius = newRadius;
    }

    /**
     * Function name: setCenterPoint
     * set the center Point of the ball.
     * The Input:
     *
     * @param newCenterPoint - the new point.
     */
    public void setCenterPoint(final Point newCenterPoint) {
        this.centerPoint = newCenterPoint;
    }

    /**
     * Function name: getCenterPoint
     * return the center Point of the ball.
     * The Input:
     *
     * @return the center point of the ball (Point object).
     */
    public Point getCenterPoint() {
        return this.centerPoint;
    }


    /**
     * Return color of the ball (Color type).
     * The Input: no input.
     *
     * @return the radius value of the ball.
     */
    public final java.awt.Color getColor() {
        return this.color;
    }

    /**
     * function name: setColor
     * void function.
     * set the color of the ball (Color type).
     * The Input:
     *
     * @param newColor - color to set to. Color type.
     */
    public void setColor(final Color newColor) {
        this.color = newColor;
    }


    /**
     * The function draw the ball on the given DrawSurface.
     * void function.
     *
     * @param surface  - input DrawSurface that helps us to draw the ball object
     *                 about him.
     * @param newColor - new color of the circle to draw.
     */
    // draw the ball on the given DrawSurface
    public void drawOn(final DrawSurface surface,
                       final Color newColor) {
        surface.setColor(newColor);
        surface.fillCircle(this.getX(), this.getY(), this.getSize());
    }



    // functions for the velocity of the ball

    /**
     * The function sets the velocity of the ball
     * void function.
     *
     * @param v - Velocity object, the function set the velocity of
     *          the ball according the input object v.
     */
    public void setVelocity(final Velocity v) {
        this.ballVelocity = v;
    }

    /**
     * The function also sets the velocity of the ball
     * void function.
     *
     * @param dx - the change of x values.
     * @param dy - the change of y values.
     */
    public void setVelocity(final double dx, final double dy) {
        this.ballVelocity.setDx(dx);
        this.ballVelocity.setDy(dy);
    }

    /**
     * The function returns the velocity of the ball.
     *
     * @return Velocity object - the velocity of the ball.
     */
    public Velocity getVelocity() {
        return this.ballVelocity;
    }

     /** The function moves one step.
     *
     * @param screenWidth  - the width of the screen.
     * @param screenLength - the length of the screen.
     * @param startPoint   - a point object that represents the
     *                     start point of the zone.
     */
    public void moveOneStep(final double screenWidth,
                            final double screenLength, final Point startPoint) {

        if (this.centerPoint.getX() - startPoint.getX()
                >= screenWidth - this.radius
                && this.getVelocity().getDx() > 0) {
            this.setCenterPoint(new Point(screenWidth - this.radius
                    + startPoint.getX(), this.centerPoint.getY()));
            this.setVelocity(-this.getVelocity().getDx(),
                    this.getVelocity().getDy());
//this.centerPoint = this.getVelocity().applyToPoint(this.centerPoint);
        }
        if (this.centerPoint.getX() <= this.radius + startPoint.getX()
                && this.ballVelocity.getDx() < 0) {
            this.setCenterPoint(new Point(this.radius + startPoint.getX(),
                    this.centerPoint.getY()));
            this.setVelocity(-this.getVelocity().getDx(),
                    this.getVelocity().getDy());
///this.centerPoint = this.getVelocity().applyToPoint(this.centerPoint);
        }
        if (this.centerPoint.getY() - startPoint.getY()
                >= screenLength - this.radius
                && this.ballVelocity.getDy() > 0) {
            this.setCenterPoint(new Point(this.centerPoint.getX(),
                    screenLength - this.radius
                            + startPoint.getY()));
            this.setVelocity(this.getVelocity().getDx(),
                    -this.getVelocity().getDy());
//this.centerPoint = this.getVelocity().applyToPoint(this.centerPoint);
        }
        if (this.centerPoint.getY() <= this.radius + startPoint.getY()
                && this.ballVelocity.getDy() < 0) {
            this.setCenterPoint(new Point(this.centerPoint.getX(),
                    this.radius + startPoint.getY()));
            this.setVelocity(this.getVelocity().getDx(),
                    -this.getVelocity().getDy());
//this.centerPoint = this.getVelocity().applyToPoint(this.centerPoint);
        }
        this.centerPoint = this.getVelocity().applyToPoint(this.centerPoint);
// when it hits the border to the left or to the right,
//it should change its horizontal direction
    }


} // end of class.

