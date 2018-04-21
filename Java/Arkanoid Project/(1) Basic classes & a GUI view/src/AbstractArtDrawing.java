//package secondass;

import java.awt.Color;
import java.util.Random;

import biuoop.DrawSurface;
import biuoop.GUI;


/**
 * This program is an AbstractArtDrawing object.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-03-30
 */
public class AbstractArtDrawing {

    /**
     * Function name : generateRandomLine.
     * Generate random line and then return it.
     * input: no input.
     *
     * @return a random Line object.
     */
    public Line generateRandomLine() {

        Random rand = new Random(); // create a random-number generator

        // get random integers numbers in range 1-400 or 1 - 300
        double x1 = 1 + (400 - 1) * rand.nextDouble();
        double y1 = 1 + (300 - 1) * rand.nextDouble();
        double x2 = 1 + (400 - 1) * rand.nextDouble();
        double y2 = 1 + (300 - 1) * rand.nextDouble();
        return new Line(x1, y1, x2, y2);
    }

    /**
     * Function name : drawLine.
     * This function draw input line.
     * input:
     *
     * @param l - is a line object that we will draw.
     * @param d - DrawSurface object that help us drawing the input line.
     */
    public void drawLine(final Line l, final DrawSurface d) {
        // now I will draw this input line.
        // casting because draeLine function gets 4 integers.

        d.drawLine((int) (l.start().getX()), (int) (l.start().getY()),
                (int) (l.end().getX()), (int) (l.end().getY()));
    }

    /**
     * Function name : displayRandomLines.
     * This function draw 10 random lines.
     * no input
     *
     * @return array of 10 random lines.
     */
    public Line[] displayRandomLines() {
        AbstractArtDrawing example = new AbstractArtDrawing();
        GUI gui = new GUI("Random Circles Example", 400, 300);
        DrawSurface d = gui.getDrawSurface();
        Line[] linesArray = new Line[10]; // array of random lines.

        // for loop to generate 10 lines and draw them on the screen.
        for (int index = 0; index < 10; index++) {
            Line randomLine = example.generateRandomLine();
            // now we will save all the random lines in array of lines
            linesArray[index] = randomLine;
            example.drawLine(randomLine, d);
            d.setColor(Color.BLACK);
        }
        displayMiddlePoint(linesArray, d);
        displayIntersectionRandomLines(linesArray, d);
        gui.show(d);
        return linesArray;
    }

    /**
     * Function name : displayMiddlePoint.
     * This function draw 10 middle points of 10 random lines.
     *
     * @param randomLinesArray - random 10 lines.
     * @param d                - DrawSurface object, helps to set color to blue
     *                         and to draw circles
     *                         (that represent the middle points).
     */
    public final void displayMiddlePoint(final Line[] randomLinesArray,
                                         final DrawSurface d) {
        for (int index = 0; index < randomLinesArray.length; index++) {
            Point middle = randomLinesArray[index].middle();
            d.setColor(Color.BLUE);
            d.fillCircle((int) middle.getX(), (int) middle.getY(), 3);
        }

    }

    /**
     * Function name : displayIntersectionRandomLines.
     * This function draws the intersection points between the random lines
     * that she gets as input to this function.
     *
     * @param randomLinesArray - random 10 lines.
     * @param d                - DrawSurface object, helps to set color to red
     *                         and to draw circles
     *                         (that represent the intersection points).
     */
    public void displayIntersectionRandomLines(final Line[] randomLinesArray,
                                               final DrawSurface d) {
        // nested loop because we want to check intersection point of each line
        // with all the other lines in the random lines array.
        int xIntersectionPoint, yIntersectionPoint;
        for (int index = 0; index < randomLinesArray.length; index++) {
            for (int i = index + 1; i < randomLinesArray.length; i++) {
                Point intersectionPoint = randomLinesArray[index].
                        intersectionWith(randomLinesArray[i]);
                if (intersectionPoint != null) {
                    d.setColor(Color.RED);
                    xIntersectionPoint = (int) intersectionPoint.getX();
                    yIntersectionPoint = (int) intersectionPoint.getY();
                    d.fillCircle(xIntersectionPoint, yIntersectionPoint, 3);
                }
            }
        }
    }

    /**
     *@param args input from user
     */
    public static void main(final String[] args) {

        AbstractArtDrawing example = new AbstractArtDrawing();
        example.displayRandomLines();
        //Line l1 = new Line (1, 1, 99, 99);
        //Line l2 = new Line (-9, -89, 22, 9);

    }
}
