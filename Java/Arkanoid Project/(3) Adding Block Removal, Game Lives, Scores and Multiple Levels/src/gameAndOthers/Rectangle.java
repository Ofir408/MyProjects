//package threeass;

import java.util.ArrayList;
import java.util.List;

//import secondass.Line;
//import secondass.Point;
/**
 * This program is a Rectangle object.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-04-22
 */

/**
 * This program is a Rectangle object.
 */
public class Rectangle {
    /**
     * upper Left point of the rectangle.
     */
    private Point upperLeft;
    /**
     * The width of the rectangle.
     */
    private double width;
    /**
     * The height of the rectangle.
     */
    private double height;

    /**
     * This is constructor function of rectangle object.
     * The Input:
     *
     * @param upperPointLeft - upper Left point of the rectangle (Point object).
     * @param recWidth       - the width of the rectangle.
     * @param recHeight      - the height of the rectangle.
     *                       The Function Operation: The function initializes
     *                       two input values in the new Rectangle object.
     */
    public Rectangle(final Point upperPointLeft, final double recWidth,
                     final double recHeight) {
        this.upperLeft = upperPointLeft;
        this.width = recWidth;
        this.height = recHeight;
    }

    // helper function

    /**
     * use the current Rectangle (this object) and returns.
     * a list of his four points
     *
     * @return a list of four points of the current rectangle.
     */
    public List<Point> rectanglePoints() {
        //ArrayList<Point> object = new ArrayList<Point>();
        //List intersectionPoints = new ArrayList();
        List<Point> rectanglePoints = new ArrayList<Point>();
        double x = this.upperLeft.getX(); // of upperLeft point in rectangle
        double y = this.upperLeft.getY(); // of upperLeft point in rectangle
        rectanglePoints.add(new Point(x, y));
        rectanglePoints.add(new Point(x + width, y));
        rectanglePoints.add(new Point(x + width, y - height));
        rectanglePoints.add(new Point(x, y - height));
        return rectanglePoints;

    }

    /**
     * use the current Rectangle (this object) and returns.
     * a list of his lines.
     *
     * @return a list of four lines of the current rectangle.
     */
    public List<Line> rectangleLines() {
        List<Point> rectanglePoints = new ArrayList<Point>();
        rectanglePoints = this.rectanglePoints();
        List<Line> rectangleLines = new ArrayList<Line>();
        rectangleLines.add(new Line(rectanglePoints.get(0),
                rectanglePoints.get(1)));
        rectangleLines.add(new Line(rectanglePoints.get(1),
                rectanglePoints.get(2)));
        rectangleLines.add(new Line(rectanglePoints.get(2),
                rectanglePoints.get(3)));
        rectangleLines.add(new Line(rectanglePoints.get(3),
                rectanglePoints.get(0)));
        return rectangleLines;
    }


    // Return a (possibly empty) List of intersection points
    // with the specified line.

    /**
     * use the current Rectangle (this object) and input line.
     * to check intersection Points. a list of his lines.
     *
     * @param line - input Line object to check the intersection points
     *             of this line with the current rectangle.
     * @return a list of intersection Points objects between the
     * current Rectangle and the input line
     * so it returns a (possibly empty) List of intersection points.
     */
    public List<Point> intersectionPoints(final Line line) {
        List<Point> intersectionPoints = new ArrayList<Point>();
        List<Line> rectangleLines = this.rectangleLines();
        // now we will check intersection of the input line with every
        // line in the current rectangle.
        for (int index = 0; index < rectangleLines.size(); index += 1) {
            if (line.isIntersecting(rectangleLines().get(index))) {
                // we will add this intersection point to the list
                intersectionPoints.add(line.intersectionWith(
                        rectangleLines().get(index)));
            }
        }
        return intersectionPoints;
    }

    // Return the width and height of the rectangle

    /**
     * @return the width of the current rectangle.
     */
    public double getWidth() {
        return this.width;
    }

    /**
     * @return the height of the current rectangle.
     */
    public double getHeight() {
        return this.height;
    }

    /**
     * @return the upper-left point of the current rectangle.
     */
    public Point getUpperLeft() {
        return this.upperLeft;
    }


}
