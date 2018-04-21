//package secondass;

/**
 * This program is a Velocity object.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-04-04
 */
public class Velocity {
    /**
     * the change of x.
     */
    private double dx;
    /**
     * the change of the x.
     */
    private double dy;

    /**
     * the change of y.
     */

    // constructor

    /**
     * This is constructor function of Velocity object. The Input:
     *
     * @param newDx - the change of x.
     * @param newDy - the change of y. The Function Operation: The function
     *              initializes two input values in the new Velocity object.
     */
    public Velocity(final double newDx, final double newDy) {
        this.dx = newDx;
        this.dy = newDy;
    }

    /**
     * This is more constructor function of Velocity object. The Input:
     *
     * @param angle - the angle of the speed vector - double number.
     * @param speed - the speed of the ball - double number. The Function
     *              Operation: The function initializes two input values in the
     *              new Velocity object after calculating dx and dy using Math
     *              package.
     * @return new Velocity object according the inputs values.
     */
    public static Velocity fromAngleAndSpeed(final double angle, final double speed) {
        double dx = speed * Math.sin(Math.toRadians(angle));
        double dy = -speed * Math.cos(Math.toRadians(angle));
        return new Velocity(dx, dy);
    }

    // functions of Velocity object.

    /**
     * Take a point with position (x,y) and return a new point. The Input:
     *
     * @param p - input point.
     * @return the new point after adding to the input point: return a new point
     * with position (x+dx, y+dy).
     */
    public Point applyToPoint(final Point p) {
        return new Point(p.getX() + this.dx, p.getY() + this.dy);
    }

    /**
     * return dx change of the Velocity. The Input: no input
     *
     * @return dx of the Velocity.
     */
    public double getDx() {
        return this.dx;
    }

    /**
     * return dy change of the Velocity. The Input: no input
     *
     * @return dy of the Velocity.
     */
    public double getDy() {
        return this.dy;
    }

    /**
     * void function. The Input:
     *
     * @param newDx - the new change of dx.
     */
    public void setDx(final double newDx) {
        this.dx = newDx;
    }

    /**
     * void function. The Input:
     *
     * @param newDy - the new change of dy.
     */
    public void setDy(final double newDy) {
        this.dy = newDy;
    }

    /**
     * Name : getSpeed. The Input:no input
     *
     * @return speed result of the current ball.
     */
    public double getSpeed() {
        double speedResult = Math.sqrt(Math.pow(this.dx, 2) + Math.pow(this.dy, 2));
        return speedResult;
    }

    /**
     * compareVelocities. The Input:
     *
     * @param other - the second Velocities.
     * @return 1 if the first Velocity bigger than the other. return 2 if the
     * second Velocity bigger than the other. return 0 if they equal.
     */
    public int compareVelocities(final Velocity other) {
        double firstSpeed = this.getSpeed();
        double secondSpeed = other.getSpeed();
        if (firstSpeed > secondSpeed) {
            return 1;
        }
        if (firstSpeed < secondSpeed) {
            return 2;
        }
        return 0; // Otherwise - they are equal.
    }

    /**
     * to string.
     */
    public void printVelocity() {
        System.out.println(" dx is: " + this.dx + " dy is : " + this.dy);
    }

    /**
     * @return angle of this velocity.
     */
    public double getAngle() {
        return (90 - Math.toDegrees(Math.atan(-this.dy / this.dx))) % 360;
    }
}
