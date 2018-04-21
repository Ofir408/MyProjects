//package secondass;

import java.util.List;

//import threeass.Rectangle;

/**
 * This program is a line object.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-03-30
 */

public class Line {
    /**
     * start is a field of Line, it's a Point object.
     */
    private Point start;
    /**
     * end is a field of Line, it's a Point object.
     */
    private Point end;

    // constructor functions

    /**
     * This is constructor function of Line object.
     * The Input:
     *
     * @param startPoint is a Point object
     * @param endPoint   is a Point object
     *                   The Function Operation: The function initializes
     *                   two input values in the new Line object.
     */
    public Line(final Point startPoint, final Point endPoint) {
        this.start = startPoint;
        this.end = endPoint;
    }

    // second constructor function:

    /**
     * This is constructor function of Line object.
     * The Input:
     *
     * @param x1 is a double number
     * @param y1 is a double number
     * @param x2 is a double number
     * @param y2 is a double number
     *           The Function Operation: The function builds two points
     *           and then initializes the new Line object with them.
     */
    public Line(final double x1, final double y1, final double x2,
                final double y2) {
        this.start = new Point(x1, y1);
        this.end = new Point(x2, y2);
    }

    /**
     * Return the length of the line
     * The Input: no input.
     *
     * @return the length of the Line - it's the distance between two points
     * (start point and end point).
     */
    public double length() {
        return this.start.distance(this.end);
    }

    /**
     * Returns the middle point of the line
     * The Input: no input
     * The Function Operation: computes the middle point of the line
     * and return it.
     *
     * @return the middle point of the line - returns Point object.
     */
    public Point middle() {
        double xSum = this.start.getX() + this.end.getX();
        double ySum = this.start.getY() + this.end.getY();
        return new Point(xSum / 2, ySum / 2);
    }

    /**
     * Returns the start point of the line.
     * The Input: no input.
     *
     * @return an object - the start point of the Line object.
     */
    public Point start() {
        return this.start;
    }

    /**
     * Returns the end point of the line
     * The Input: no input.
     *
     * @return an object - the end point of the Line object.
     */
    public Point end() {
        return this.end;
    }

    /**
     * Helper function that gets an Incline and return b
     * in the equation y = mx + b.
     *
     * @param p1 - it's a Point object of the current line.
     * @param m  - the Incline of the current line.
     * @return b in the  equation of the current line: y = mx + b.
     */

    public double getBInEquation(final double m, final Point p1) {
        return (p1.getY() - m * p1.getX());
    }

    /**
     * Returns true if the lines intersect, false otherwise.
     * The Input:
     *
     * @param other - it's a line object.
     * @return boolean : true if the current line and and input
     * line intersects, false otherwise.
     * In the case of infinite intersection points, return False.
     */
    public boolean isIntersecting(final Line other) {

        // if the lines are parallel - returns false
        // because there is no option to Intersect Point.
        // now we assume that there is a intersect Point
        // because we know they now parallel if there was no return
        // so we will find the Point and check if she Exists on both lines.

        if ((this.checkIfParallel(other))
                || (!this.checkIfInputPointOnLines(other,
                this.findIntersection(other)))) {
            return false;
        }


        return true; // because we checked all the cases.
    }

    /**
     * Returns the intersection point if the lines intersect,
     * and null otherwise.
     * The Input:
     *
     * @param other - it's a line object.
     * @return Point - Returns the intersection point if the lines intersect,
     * In the case that there is no intersection point return Null.
     */
    public Point intersectionWith(final Line other) {

        if (!this.isIntersecting(other)) {
            return null; // because it means there is no intersection point
        }
        // else - compute and then return the point.
        return this.findIntersection(other);
    }

    /**
     * Returns intersection point with the assumption that
     * the two lines does not parallel.
     * The Input:
     *
     * @param other - it's a line object.
     * @return Point - Returns the intersection point.
     * it must be a point like that because out assumption.
     */
    public Point findIntersection(final Line other) {
        double m1, m2, b1, b2;


        if (this.end.getY() - this.start.getY() == 0) {
            if (other.end.getX() == other.start.getX()) {
                findIntersForEqualsYAndX(this, other);
            } else {
                return findIntersectionForEqualsY(this, other);
            }
        }
        if (other.end.getY() - other.start.getY() == 0) {
            if (this.end.getX() == this.start.getX()) {
                return findIntersForEqualsYAndX(other, this);
            }
            return findIntersectionForEqualsY(other, this);
        }

        // in order do not divide in zero.
        if (this.end.getX() - this.start.getX() == 0) {
            if (other.end.getY() == other.start.getY()) {
                return findIntersForEqualsYAndX(other, this);
            }
            return findIntersectionForEqualsX(this, other);
        } else {
            m1 = (this.end.getY() - this.start.getY())
                    / (this.end.getX() - this.start.getX());
        }

        if (other.end.getX() - other.start.getX() == 0) {
            if (this.end.getY() == this.start.getY()) {
                return findIntersForEqualsYAndX(this, other);
            }
            return findIntersectionForEqualsX(other, this);
        } else {
            m2 = (other.end.getY() - other.start.getY())
                    / (other.end.getX() - other.start.getX());
        }

        b1 = this.getBInEquation(m1, this.start);
        b2 = this.getBInEquation(m2, other.start);
        // now I will find if there is x for them intersect point.
        // assume that we have a intersect point between this two lines.
        double xIntersct = ((b2 - b1) / (m1 - m2));
        // now we have x, just need to find y
        // we will take a point and dispose it in the current line
        double yIntersct = (m1 * xIntersct) + b1;
        return new Point(xIntersct, yIntersct);
    }

    /**
     * This function gets a line and a point and return
     * true if the point is on the current line
     * and also is on the input lines.
     * The Input:
     *
     * @param other      - it's a line object.
     * @param inputPoint - it's a point object.
     * @return boolean - Returns true if the point is on
     * both the lines.
     */
    public boolean checkIfInputPointOnLines(final Line other,
                                            final Point inputPoint) {

       /* if (inputPoint.getX() >= this.minAndMaxPoints()[0].getX()
                && inputPoint.getX() <= this.minAndMaxPoints()[1].getX()) {
            if (inputPoint.getX() >= other.minAndMaxPoints()[0].getX()
                    && inputPoint.getX() <= other.minAndMaxPoints()[1].getX()
            && inputPoint.getY() >= this.minAndMaxPoints()[0].getY()
            && inputPoint.getY() <= this.minAndMaxPoints()[1].getY()
            && inputPoint.getY() >= other.minAndMaxPoints()[0].getY()
            && inputPoint.getY() <= other.minAndMaxPoints()[1].getY()) */
        //{

        //if ((inputPoint.getY() == (m1 * inputPoint.getX() + b1))
        //        && (inputPoint.getY()
        //        == (m2 * inputPoint.getX() + b2))) {
        if (this.isInputPointOnOneLine(inputPoint)
                && other.isInputPointOnOneLine(inputPoint)) {
            if (new Line(this.start, inputPoint)
                    .checkIfParallel(new Line(inputPoint, this.end))
                    && new Line(other.start, inputPoint)
                    .checkIfParallel(new Line(inputPoint, other.end))) {
                return true;
            }
        }
        return false;
    }

    /**
     * This function gets a line and returns true if the
     * current line and the input line are Parallel.
     * The Input:
     *
     * @param other - it's a line object.
     * @return true if parallel, otherwise return false.
     */
    public boolean checkIfParallel(final Line other) {
        double m1, m2;

        if (this.end.getY() - this.start.getY() == 0
                && other.end.getY() - other.start.getY() == 0) {
            return true;
        }

        if ((this.end.getX() - this.start.getX() == 0  // **** changed
                && other.end.getX() - other.start.getX() == 0)) {
            return true;
        } else {
            m1 = (this.end.getY() - this.start.getY())
                    / (this.end.getX() - this.start.getX());
        }

        if (other.end.getX() - other.start.getX() == 0) {
            m2 = 0;
        } else {
            m2 = (other.end.getY() - other.start.getY())
                    / (other.end.getX() - other.start.getX());
        }
        if (Math.abs(m1 - m2) <= 0.00000001) { // because of rounding digits.
            return true;
        }
        return false;
    }


    /**
     * Function name : minAndMaxPoints.
     * input: no input.
     *
     * @return the max Point on the line and the min Point
     * in order to do it it return array of two Points
     * the the first index is the max Point, and the second is
     * the min Point in the current line
     */

    public Point[] minAndMaxPoints() {
        Point[] minMaxArray = new Point[2];
        if (this.start.getX() > this.end.getX()) {
            minMaxArray[0] = this.end;
            minMaxArray[1] = this.start;
        } else {
            minMaxArray[0] = this.start;
            minMaxArray[1] = this.end;
        }
        return minMaxArray;
    }

    /**
     * Function name : equals.
     * input:
     *
     * @param other - Line object.
     * @return return true is the lines are equal, false otherwise
     */
    public boolean equals(final Line other) {
        if (this.start().equals(other.start)
                && this.end().equals(other.end)) {
            return true;
        }
        return false;
    }

    /**
     * function that calls when Xstart = Xend, in order to find intersection
     * between two lines that one of them (firstLine) has Xstart = Xend.
     *
     * @param firstLine  - in this line  Xstart = Xend.
     * @param secondLine - in this line Xstart and Xend does not equal!
     * @return new Point - find the intersection
     * between two lines that one of them (firstLine) has Xstart = Xend
     * and then send him back to other function to check if this point in
     * our scope.
     */
    private static Point findIntersectionForEqualsX(final Line firstLine,
                                                    final Line secondLine) {
        double m2, b2;
        // find the equasion of the secondLine to check intersection
        m2 = (secondLine.end.getY() - secondLine.start.getY())
                / (secondLine.end.getX() - secondLine.start.getX());

        b2 = secondLine.getBInEquation(m2, secondLine.start);
        ////////////////////////// ************* check if - b2 or + b2 *** ///
        return new Point(firstLine.start.getX(), m2 * firstLine.start.getX() + b2);
    }

    /**
     * function that calls when Ystart = Yend, in order to find intersection
     * between two lines that one of them (firstLine) has Ystart = Yend.
     *
     * @param firstLine  - in this line  Ystart = Yend.
     * @param secondLine - in this line Ystart and Yend does not equal!
     * @return new Point - find the intersection
     * between two lines that one of them (firstLine) has Ystart = Yend
     * and then send him back to other function to check if this point in
     * our scope.
     */
    private static Point findIntersectionForEqualsY(final Line firstLine,
                                                    final Line secondLine) {
        double m2, b2;
        // find the equasion of the secondLine to check intersection
        m2 = (secondLine.end.getY() - secondLine.start.getY())
                / (secondLine.end.getX() - secondLine.start.getX());

        b2 = secondLine.getBInEquation(m2, secondLine.start);
        return new Point((firstLine.start.getY() - b2) / m2,
                firstLine.start.getY());
    }

    /**
     * function that calls when Ystart = Yend, in order to find intersection
     * between two lines that one of them (firstLine) has Ystart = Yend.
     *
     * @param firstLine  - in this line  Ystart = Yend.
     * @param secondLine - in this line Xstart and Xend equal!
     * @return new Point - find the intersection
     * send him back to other function to check if this point in
     * our scope.
     */
    private static Point findIntersForEqualsYAndX(final Line firstLine,
                                                  final Line secondLine) {
        return new Point(secondLine.start.getX(), firstLine.start.getY());
    }


    /**
     * Function name : closestIntersectionToStartOfLine.
     * input:
     *
     * @param rect - a Rectangle object to check intersection point with him.
     * @return If this line does not intersect with the rectangle, return null
     * Otherwise, return the closet intersection point of the rectangle
     * with the current line, the closet point to the start of the current line.
     */

    public Point closestIntersectionToStartOfLine(final Rectangle rect) {
        List<Point> intersectionPoints = rect.intersectionPoints(this);
        if (intersectionPoints.isEmpty()) {
            return null;
        } else {
            // it means there is at least one intersection point
            // so initialize the first point to be
            // the closest to start of line, then check others if there are.
            Point closetIntersectionPoint = intersectionPoints.get(0);

            for (int i = 1; i < intersectionPoints.size(); i += 1) {
                // find the closet intersection point to
                //the start of current line.
                double closetIntersLength = new Line(this.start,
                        closetIntersectionPoint).
                        length();

                if (new Line(this.start, intersectionPoints.get(i)).
                        length() < closetIntersLength) {
                    closetIntersectionPoint = intersectionPoints.
                            get(i);
                }

            }
            return closetIntersectionPoint;
        }
    } // end of function.

    /**
     * Function name : closestPointToLine.
     * input:
     *
     * @param p1 - the first point.
     * @param p2 - the second point.
     * @return the point that more close to the start of the current
     * (this) line.
     */
    public Point closestPointToLine(final Point p1, final Point p2) {
        if (new Line(this.start, p1).length() < new Line(
                this.start, p2).length()) {
            return p1;
        }
        return p2;
    }

    /**
     * check if input point is on the current line (this).
     *
     * @param inputP - the input point to check with.
     * @return true if the input point on the current line
     * otherwise - return false
     */
    public boolean isInputPointOnOneLine(final Point inputP) {
        double xMax, xMin, yMax, yMin;
        if (this.start.getX() > this.end.getX()) {
            xMax = this.start.getX();
            xMin = this.end.getX();
        } else {
            xMin = this.start.getX();
            xMax = this.end.getX();
        }
        if (this.start.getY() > this.end.getY()) {
            yMax = this.start.getY();
            yMin = this.end.getY();
        } else {
            yMin = this.start.getY();
            yMax = this.end.getY();
        }
        if (inputP.getX() >= xMin && inputP.getX() <= xMax
                && inputP.getY() >= yMin && inputP.getY() <= yMax) {
            return true;
        }
        return false;
    }

}
