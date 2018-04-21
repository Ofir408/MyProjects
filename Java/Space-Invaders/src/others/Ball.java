
//package secondass;

import java.awt.Color;

import biuoop.DrawSurface;
//import threeass.CollisionInfo;
//import threeass.Game;
//import threeass.GameEnvironment;
//import threeass.Sprite;

/**
 * This program is a Ball object.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-04-03
 */
public class Ball implements Sprite {

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
    private GameEnvironment gameEnvironment;
    /**
     * if the ball produce from an Alien object, it's true.
     * Otherwise - false.
     */
    private boolean isOfAlien;

    // functions:

    /**
     * This is constructor function of Ball object. The Input:
     *
     * @param center    - the place of the ball - it's a point object.
     * @param newRadius - the radius of the ball - an integer number.
     * @param newColor  - the color of the ball. The Function Operation: The function
     *                  initializes two input values in the new Ball object.
     */
    public Ball(final Point center, final int newRadius, final java.awt.Color newColor) {
        this.centerPoint = center;
        this.radius = newRadius;
        this.color = newColor;
        this.ballVelocity = new Velocity(0.0, 0.0);
    }

    /**
     * This is another constructor function of Ball object. The Input:
     *
     * @param xValue    - the x value of the ball - it's an integer.
     * @param yValue    - the y value of the ball - it's an integer.
     * @param newRadius - the radius of the ball - an integer number.
     * @param newColor  - the color of the ball. The Function Operation: The function
     *                  create new point with the input and then initializes two input
     *                  values in the new Ball object.
     */

    public Ball(final int xValue, final int yValue, final int newRadius, final Color newColor) {
        this.centerPoint = new Point(xValue, yValue);
        this.radius = newRadius;
        this.color = newColor;
        this.ballVelocity = new Velocity(0.0, 0.0);
    }

    /**
     * This is another constructor function of Ball object. The Input:
     *
     * @param center    - the place of the ball - it's a point object.
     * @param newRadius - the radius of the ball - an integer number.
     * @param newColor  - the color of the ball. The Function Operation: The function
     *                  initializes two input values in the new Ball object.
     * @param v         - the velocity of the ball.
     * @param g         - the GameEnvironment of the ball.
     */
    public Ball(final Point center, final int newRadius, final java.awt.Color newColor, Velocity v, GameEnvironment g) {
        this.centerPoint = center;
        this.radius = newRadius;
        this.color = newColor;
        this.ballVelocity = v;
        this.gameEnvironment = g;
    }

    // Accessory functions.

    /**
     * Return x value of the center point of the ball. The Input: no input,
     * casting to integers because the function in point return double and here
     * we return int.
     *
     * @return the x value of the center point of the ball.
     */
    public final int getX() {
        return (int) this.centerPoint.getX();
    }

    /**
     * Return y value of the center point of the ball. The Input: no input,
     * casting to integers because the function in point return double and here
     * we return int.
     *
     * @return the y value of the center point of the ball.
     */
    public int getY() {
        return (int) this.centerPoint.getY();
    }

    /**
     * Return radius value of the ball. The Input: no input.
     *
     * @return the radius value of the ball.
     */
    public int getSize() {
        return this.radius;
    }

    /**
     * @return gameEnvironment.
     */
    public GameEnvironment getGame() {
        return this.gameEnvironment;
    }

    /**
     * Function name: setRadious set the radius value of the ball. The Input:
     *
     * @param newRadius - the new radius.
     */
    public void setRadius(final int newRadius) {
        this.radius = newRadius;
    }

    /**
     * Function name: setCenterPoint set the center Point of the ball. The
     * Input:
     *
     * @param newCenterPoint - the new point.
     */
    public void setCenterPoint(final Point newCenterPoint) {
        this.centerPoint = newCenterPoint;
    }

    /**
     * Function name: getCenterPoint return the center Point of the ball. The
     * Input:
     *
     * @return the center point of the ball (Point object).
     */
    public Point getCenterPoint() {
        return this.centerPoint;
    }

    /**
     * Return color of the ball (Color type). The Input: no input.
     *
     * @return the radius value of the ball.
     */
    public final java.awt.Color getColor() {
        return this.color;
    }

    /**
     * function name: setColor void function. set the color of the ball (Color
     * type). The Input:
     *
     * @param newColor - color to set to. Color type.
     */
    public void setColor(final Color newColor) {
        this.color = newColor;
    }

    /**
     * @param v - new Velocity to set to original vel.
     *
    public void setOriginalVel(Velocity v) {
    this.originalVel = v;
    }*/

    /**
     * @param newGameEnvironment - object to set as the current game environment.
     */
    public void setGameEnvironment(final GameEnvironment newGameEnvironment) {
        this.gameEnvironment = newGameEnvironment;
    }

    /**
     * The function draw the ball on the given DrawSurface. void function.
     *
     * @param surface  - input DrawSurface that helps us to draw the ball object
     *                 about him.
     * @param newColor - new color of the circle to draw.
     */
    // draw the ball on the given DrawSurface
    public void drawOn(final DrawSurface surface, final Color newColor) {
        surface.setColor(newColor);
        surface.fillCircle(this.getX(), this.getY(), this.getSize());
        surface.setColor(Color.BLACK);
        surface.drawCircle(this.getX(), this.getY(), this.getSize()); // check
    }

    /**
     * The function draw the ball on the given DrawSurface. void function.
     *
     * @param surface - input DrawSurface that helps us to draw the ball object
     *                about him.
     */
    // draw the ball on the given DrawSurface
    @Override
    public void drawOn(final DrawSurface surface) {
        surface.setColor(this.color);
        surface.fillCircle(this.getX(), this.getY(), this.getSize());
        surface.setColor(Color.BLACK);
        surface.drawCircle(this.getX(), this.getY(), this.getSize()); // check
    }

    // functions for the velocity of the ball

    /**
     * The function sets the velocity of the ball void function.
     *
     * @param v - Velocity object, the function set the velocity of the ball
     *          according the input object v.
     */
    public void setVelocity(final Velocity v) {
        this.ballVelocity = v;
    }

    /**
     * The function also sets the velocity of the ball void function.
     *
     * @param dx - the change of x values.
     * @param dy - the change of y values.
     */
    public void setVelocity(final double dx, final double dy) {
        this.ballVelocity.setDx(dx);
        this.ballVelocity.setDy(dy);
    }

    /**
     * the ball from Alien, set to true.
     */
    public void setToAlienBall() {
        this.isOfAlien = true;
    }

    /**
     * @return True if the ball from Alien. Otherwise false.
     */
    public boolean getIsBallFromAlien() {
        return this.isOfAlien;
    }

    /**
     * The function returns the velocity of the ball.
     *
     * @return Velocity object - the velocity of the ball.
     */
    public Velocity getVelocity() {
        return this.ballVelocity;
    }

    /**
     * The function computes the trajectory of the ball (his movement in one
     * step according dx, dy velocity.
     *
     * @return Line object - the trajectory of the ball one movement.
     */
    public Line ballTrajectory() {
        if (this.ballVelocity.getDx() == 0 && this.ballVelocity.getDy() == 0) {
            return null; // no movement.
        }
        return new Line(this.centerPoint.getX(), this.centerPoint.getY(),
                this.centerPoint.getX() + this.ballVelocity.getDx(),
                this.centerPoint.getY() + this.ballVelocity.getDy());
    }

    /**
     * adding the ball and the block to the game, with calling the appropriate
     * game methods.
     *
     * @param g - the game object, that we will add to him the current ball.
     */
    @Override
    public void addToGame(final GameLevel g) {
        g.addSprite(this);
    }

    /**
     * What should timePassed do? For the ball, it should move one step.
     *
     * @param dt - the amount of seconds passed since the last call.
     */
    @Override
    public void timePassed(double dt) {

        // update according support dt.

        this.moveOneStep();

    }

    /**
     * Update of The function moves one step. 1) compute the ball trajectory
     * (the trajectory is "how the ball will move without any obstacles" -- its
     * a line starting at current location, and ending where the velocity will
     * take the ball if no collisions will occur).
     * <p>
     * 2) Check (using the game environment) if moving on this trajectory will
     * hit anything.
     * <p>
     * 2.1) If no, then move the ball to the end of the trajectory.
     * <p>
     * 2.2) Otherwise (there is a hit): 2.2.2) move the ball to "almost" the hit
     * point, but just slightly before it. 2.2.3) notify the hit object (using
     * its hit() method) that a collision occurred. 2.2.4) update the velocity
     * to the new velocity returned by the hit() method.
     */
    public void moveOneStep() {

        CollisionInfo infoCollision = this.gameEnvironment.getClosestCollision(this.ballTrajectory());

        if (infoCollision == null || (this.isOfAlien && infoCollision.collisionObject() instanceof Alien)) {
            // it means no collision in this movement step make the move
            // to the end of the current trajectory.
            this.centerPoint = this.ballVelocity.applyToPoint(this.centerPoint);
            return;
        }

        Point collisionPoint = infoCollision.collisionPoint();
        moveToClosePoint(collisionPoint);

        this.setVelocity(infoCollision.collisionObject().hit(this,
                infoCollision.collisionPoint(), this.ballVelocity));

    }

    /**
     * helper function number 2.
     *
     * @param c - is the intersection point between the ball and the other
     *          object (block, paddle..).
     */
    private void moveToClosePoint(final Point c) {
        double x = 0.01;
        if (this.ballVelocity.getDx() >= 0) {
            if (this.ballVelocity.getDy() >= 0) {
                this.centerPoint = new Point(c.getX() - x, c.getY() - x);
            } else {
                this.centerPoint = new Point(c.getX() - x, c.getY() + x);
            }
        } else {
            if (this.ballVelocity.getDy() >= 0) {
                this.centerPoint = new Point(c.getX() + x, c.getY() - x);
            } else {
                this.centerPoint = new Point(c.getX() + x, c.getY() + x);
            }
        }
    }

    /**
     * The function moves one step.
     *
     * @param screenWidth  - the width of the screen.
     * @param screenLength - the length of the screen.
     * @param startPoint   - a point object that represents the start point of the zone.
     */
    public void moveOneStep(final double screenWidth, final double screenLength, final Point startPoint) {

        if (this.centerPoint.getX() - startPoint.getX() >= screenWidth - this.radius
                && this.getVelocity().getDx() > 0) {
            this.setCenterPoint(new Point(screenWidth - this.radius + startPoint.getX(), this.centerPoint.getY()));
            this.setVelocity(-this.getVelocity().getDx(), this.getVelocity().getDy());
            // this.centerPoint =
            // this.getVelocity().applyToPoint(this.centerPoint);
        }
        if (this.centerPoint.getX() <= this.radius + startPoint.getX() && this.ballVelocity.getDx() < 0) {
            this.setCenterPoint(new Point(this.radius + startPoint.getX(), this.centerPoint.getY()));
            this.setVelocity(-this.getVelocity().getDx(), this.getVelocity().getDy());
            /// this.centerPoint =
            /// this.getVelocity().applyToPoint(this.centerPoint);
        }
        if (this.centerPoint.getY() - startPoint.getY() >= screenLength - this.radius
                && this.ballVelocity.getDy() > 0) {
            this.setCenterPoint(new Point(this.centerPoint.getX(), screenLength - this.radius + startPoint.getY()));
            this.setVelocity(this.getVelocity().getDx(), -this.getVelocity().getDy());
            // this.centerPoint =
            // this.getVelocity().applyToPoint(this.centerPoint);
        }
        if (this.centerPoint.getY() <= this.radius + startPoint.getY() && this.ballVelocity.getDy() < 0) {
            this.setCenterPoint(new Point(this.centerPoint.getX(), this.radius + startPoint.getY()));
            this.setVelocity(this.getVelocity().getDx(), -this.getVelocity().getDy());
            // this.centerPoint =
            // this.getVelocity().applyToPoint(this.centerPoint);
        }
        this.centerPoint = this.getVelocity().applyToPoint(this.centerPoint);
        // when it hits the border to the left or to the right,
        // it should change its horizontal direction
    }

    /**
     * Remove the current ball from the Game (input g, the Game object).
     *
     * @param g - the current game, that we want to remove the ball from him.
     */
    public void removeFromGame(GameLevel g) {
        g.removeSprite(this);
    }

} // end of class.
