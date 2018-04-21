//package secondass;

/**
 * This program is a point object.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-03-30
 */
public class Point {
    /**
     * x is a field of Point, a double number.
     */
    private double x;
    /**
     * y is a field of Point, a double number.
     */
    private double y;

    // constructor function

    /**
     * This is constructor function of Point object.
     * The Input:
     *
     * @param newX is a double number
     * @param newY is a double number
     *             The Function Operation: The function initializes
     *             two input values in the new Point object.
     */
    public Point(final double newX, final double newY) {
        this.x = newX;
        this.y = newY;
    }

    /**
     * Function name: distance.
     * The Input:
     *
     * @param other - Point object
     *              The Function Operation: The function computes the
     *              distance between the input point the the current object
     *              and returns the result.
     * @return the distance between the input point and the current
     * Point object.
     */
    public double distance(final Point other) {
        double result = Math.pow(other.x - this.x, 2)
                + Math.pow(other.y - this.y, 2);
        return Math.sqrt(result);
    }

    // equals -- return true is the points are equal, false otherwise

    /**
     * Function name: equals.
     * The Input:
     *
     * @param other - Point object
     *              The Function Operation: The function checks if the
     *              current Point object is equal to other input Point.
     *              and returns the result.
     * @return true is the points are equal, false otherwise
     */
    public boolean equals(final Point other) {
        if (other.x == this.x && other.y == this.y) {
            return true;
        }
        return false;
    }

    /**
     * Function name: getX.
     * The Input: no input.
     *
     * @return return x value of the current Point.
     */
    public double getX() {
        return this.x;
    }

    /**
     * Function name: getY.
     * The Input: no input.
     *
     * @return return y value of the current Point.
     */
    public double getY() {
        return this.y;
    }
}
