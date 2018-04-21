//package secondass;

import java.awt.Color;
import java.util.Random;

import biuoop.GUI;

/**
 * class MultipleBouncingBallsAnimation.
 */
public class MultipleBouncingBallsAnimation {

    /**
     * the width of the screen display.
     */
    private double screenWidth;
    /**
     * the length of the screen display.
     */
    private double screenLength;

    // constructor

    /**
     * This is constructor function of MultipleBouncingBallsAnimation object.
     * The Input:
     *
     * @param widthScreen  - the screen width.
     * @param lengthScreen - the screen length.
     *                     The Function Operation: The function initializes
     *                     input values in the new
     *                     MultipleBouncingBallsAnimation object.
     */
    public MultipleBouncingBallsAnimation(final double widthScreen,
                                          final double lengthScreen) {
        this.screenWidth = widthScreen;
        this.screenLength = lengthScreen;
    }

    /**
     * Function name: getScreenWidth.
     *
     * @return the screen width.
     */
    public double getScreenWidth() {
        return this.screenWidth;
    }

    /**
     * Function name: getScreenLength.
     *
     * @return the screen length.
     */
    public double getScreenLength() {
        return this.screenLength;
    }


    /**
     * Function name: stringsToInts.
     * The Function Operation: The function creates
     * new Array of Integers from the input String Array
     * and returns him.
     * The Input:
     *
     * @param numbers : Array of strings
     *                The Output: Array of Integers
     * @return new Array of Integers.
     */
    public static int[] stringsToInts(final String[] numbers) {
        int[] myIntArray = new int[numbers.length];

        try {
            for (int index = 0; index < myIntArray.length; index += 1) {
                if (Integer.parseInt(numbers[index]) > 0) {
                    assert true; // do nothing.
                } else {
                    System.out.println(" invalid input, radius should be"
                            + " more than 0 ");
                    System.exit(1);
                }
                myIntArray[index] = Integer.parseInt(numbers[index]);
            }
            return myIntArray;
        } catch (NumberFormatException e) {
            System.out.println("a problem to convert input radius to integers");
            System.exit(1);
            return null;
        }

    }
    //I do not use this function.
    /**
     * Function name: generateArrayRandomPointS.
     * The Function Operation: The function creates
     * new Array of random points and sorts the array
     * and returns him.
     * The Input:
     *
     * @param inputNumbers : the number of input number size (to balls).
     * @param widthScreen  - the width of the screen display.
     * @param lengthScreen - the length of the screen display.
     *                     The Output: Array of Integers
     * @return new Array of random points.
     *
    public Point[] generateArrayRandomPointS(final int inputNumbers,
    final double widthScreen,
    final double lengthScreen) {
    Point[] randomPointsArray = new Point[inputNumbers];

    for (int i = 0; i < inputNumbers; i += 1) {
    Point randomPoint = this.generateRandomPoint(widthScreen,
    lengthScreen, new Point(0, 0));
    randomPointsArray[i] = randomPoint;
    }
    // here we have array of random points so return it
    return randomPointsArray;
    } */

    /**
     * Function name: generateArrayRandomVelocity.
     * The Function Operation: The function creates
     * new Array of random points
     * and returns him.
     * The Input:
     *
     * @param ballSizeArray - array of radius sizes.
     * @param inputNumbers  : the number of input number size, namely, how
     *                      much random velocities we need to generate.
     * @param widthScreen   - the width of the screen display.
     * @param lengthScreen  - the length of the screen display.
     *                      The Output: Array of Integers
     * @param startPoint    - start point.
     * @return new Array of random points.
     */
    public Velocity[] generateArrayRandomVelocity(final int[] ballSizeArray,
                                                  final int inputNumbers,
                                                  final double widthScreen,
                                                  final double lengthScreen,
                                                  final Point startPoint) {
        Velocity[] randomVelocityArray = new Velocity[inputNumbers];

        for (int i = 0; i < inputNumbers; i += 1) {
            Velocity randomVelocity = this.generateVelocityAccordingRadius(
                    ballSizeArray[i]);
            randomVelocityArray[i] = randomVelocity;
        }
        // here we have array of random points so return it
        return randomVelocityArray;
    }

    /**
     * Function name: generateRandomVelocity.
     * The Function Operation: returns random Velocity (an object).
     *
     * @param radius           - the radius of the ball.
     * @param widthScreenSize  - the width of the screen display.
     * @param lengthScreenSize - the length of the screen display.
     * @param startPoint       - start point of the space.
     * @return new random Velocity.
     */
    public Velocity generateRandomVelocity(final int radius,
                                           final double widthScreenSize,
                                           final double lengthScreenSize,
                                           final Point startPoint) {
        Random rand = new Random(); // create a random-number generator
        double dx = startPoint.getX() + (widthScreenSize - 1)
                * rand.nextDouble();
        double dy = startPoint.getY() + (lengthScreenSize - 1)
                * rand.nextDouble();
        if (dx < startPoint.getX() || dy < startPoint.getY()) {
            return generateRandomVelocity(radius, widthScreenSize,
                    lengthScreenSize, startPoint);
        }
        return new Velocity(dx, dy);
    }

    /**
     * Function name: generateVelocityAccordingRadius.
     * The Function Operation: returns  Velocity (an object).
     *
     * @param radius - the radius of the ball.
     * @return new  Velocity.
     */
    public Velocity generateVelocityAccordingRadius(final int radius) {
        double v = Math.max((50 - radius) / 10, 1);
        Random rand = new Random(); // create a random-number generator
        double dx = 1 + (360 - 1)
                * rand.nextDouble();
        return Velocity.fromAngleAndSpeed(dx, v);
    }

    /**
     * Function name: generateRandomPoint.
     * The Function Operation: returns random point.
     *
     * @param widthScreenSize  - the width of the screen display.
     * @param lengthScreenSize - the length of the screen display.
     * @param ballRadius       - the radius of the ball.
     * @param startPoint       - the start point of the space.
     * @return new random point.
     */
    public Point generateRandomPoint(final double widthScreenSize,
                                     final double lengthScreenSize,
                                     final double ballRadius,
                                     final Point startPoint) {
        Random rand = new Random(); // create a random-number generator
        double x, y;
        do {
            x = startPoint.getX() + ballRadius + (widthScreenSize - 1) * rand.nextDouble();
            y = startPoint.getY() + ballRadius + (lengthScreenSize - 1) * rand.nextDouble();
        } while (x < startPoint.getX()
                || x > startPoint.getX() + widthScreenSize
                || y < startPoint.getY()
                || y > startPoint.getY()
                + lengthScreenSize);
        return new Point(x, y);
    }

    /**
     * Function name: ascSotringIntegers.
     * The Function Operation: sorts the input array
     * according Bubble Sort sorts in ascending order.
     * The Input:
     *
     * @param intArray is array of integers.
     *                 The Output: void function
     */
    public static void ascSotringIntegers(int[] intArray) {
        // algorithm sorting - Bubble sort.
        for (int i = 0; i < intArray.length; i += 1) {
            for (int j = 1; j < intArray.length; j += 1) {
                if (intArray[j - 1] > intArray[j]) {
                    swapIntegers(j, j - 1, intArray);
                }
            }
        }
    }

    /**
     * Function name: ascSotringVelocities.
     * The Function Operation: sorts the input array
     * according Bubble Sort sorts in ascending order.
     * The Input:
     *
     * @param velocitiesArray is array of Velocities.
     *                        The Output: void function
     */
    public static void ascSotringVelocities(Velocity[] velocitiesArray) {
        // algorithm sorting - Bubble sort.
        for (int i = 0; i < velocitiesArray.length; i += 1) {
            for (int j = 1; j < velocitiesArray.length; j += 1) {
                if (velocitiesArray[j - 1].
                        compareVelocities(velocitiesArray[j]) == 1) {
                    swapVelocities(j, j - 1, velocitiesArray);
                }
            }
        }
    }

    /**
     * Function name: swapIntegers.
     * The Function Operation: The function swap the value
     * in the first index in the array and the second.
     * The Input:
     *
     * @param firstIndex  - an index in the array
     * @param secondIndex - another index in the array
     * @param intArray    - array of integers
     *                    The Output: void function
     */
    public static void swapIntegers(final int firstIndex, final int secondIndex,
                                    int[] intArray) {
        int temp = intArray[firstIndex]; // temp to save the number
        intArray[firstIndex] = intArray[secondIndex];
        intArray[secondIndex] = temp;
    }

    /**
     * Function name: swapVelocities.
     * The Function Operation: The function swap the value
     * in the first index in the array and the second.
     * The Input:
     *
     * @param firstIndex      - an index in the array
     * @param secondIndex     - another index in the array
     * @param velocitiesArray - array of velocities
     *                        The Output: void function
     */
    public static void swapVelocities(final int firstIndex,
                                      final int secondIndex,
                                      Velocity[] velocitiesArray) {

        Velocity temp = velocitiesArray[firstIndex]; // temp to save
        velocitiesArray[firstIndex] = velocitiesArray[secondIndex];
        velocitiesArray[secondIndex] = temp;
    }


    /**
     * Function name: howMuchSizeAbove50.
     *
     * @param ballSizeArray - the input sizes of the balls.
     * @return the number of the input size of ball that
     * above 50.
     */
    public int howMuchSizeAbove50(final int[] ballSizeArray) {
        int counter = 0;
        for (int i = 0; i < ballSizeArray.length; i += 1) {
            if (ballSizeArray[i] > 50) {
                counter += 1;
            }
        }
        return counter;
    }

    /**
     * Function name: mixSizeAndVelocities.
     * gets ballSizeArray and randomVelocityArray and returns new array of balls
     * after initialize velocities.
     *
     * @param ballSizeArray         - the input array sizes of the balls.
     *                              it's an integer type.
     * @param randomVelocitiesArray - input array of Velocities.
     *                              void function: mix size and velocities
     *                              (array of Balls objects),
     *                              update random velocities in the array
     *                              of balls size.
     * @param startPoint            - start Point of space.
     * @return new array of balls after putting values in their fields.
     */
    public Ball[] mixSizeAndVelocities(final int[] ballSizeArray,
                                       final Velocity[] randomVelocitiesArray,
                                       final Point startPoint) {
        Ball[] newBallArray = new Ball[ballSizeArray.length];
        for (int i = 0; i < ballSizeArray.length; i += 1) {
            Ball tempBall = new Ball(0, 0, 0, Color.BLACK);
            tempBall.setVelocity(randomVelocitiesArray[i]);
            tempBall.setColor(Color.BLACK);
            tempBall.setRadius(ballSizeArray[i]);
            tempBall.setCenterPoint(
                    generateRandomPoint(this.screenWidth,
                            this.screenLength, ballSizeArray[i],
                            startPoint)); // random Point
            newBallArray[i] = tempBall;
        }
        return newBallArray;
    }

    /**
     * Function name: gettingBallFullArray.
     *
     * @param stringArray - the input sizes of the balls (strings now).
     * @param sWidth      - screen width.
     * @param sLength     - screen length.
     * @param startPoint  - start point (left down).
     * @return the full balls array after making velocities,
     * random starts points.
     */
    public Ball[] gettingBallFullArray(final String[] stringArray,
                                       final double sWidth,
                                       final double sLength,
                                       final Point startPoint) {

        this.screenLength = sLength;
        this.screenWidth = sWidth;
        int[] ballSizeArray = new int[stringArray.length];
        ballSizeArray = stringsToInts(stringArray);
        Velocity[] randomVelocityArray =
                this.generateArrayRandomVelocity(ballSizeArray,
                        ballSizeArray.length, this.screenWidth, this.screenLength,
                        startPoint);
        int i = randomVelocityArray.length
                - this.howMuchSizeAbove50(ballSizeArray);
        ascSotringIntegers(ballSizeArray);
        ascSotringVelocities(randomVelocityArray);
        final Velocity minVelocity = randomVelocityArray[0];
        for (; i < randomVelocityArray.length; i += 1) {
            // above size 50 can all have the same slow speed
            // put here the minimum speed to every above 50
            randomVelocityArray[i] = minVelocity;
        }
        Ball[] fullArrayBall = this.mixSizeAndVelocities(ballSizeArray,
                randomVelocityArray, startPoint);
        return fullArrayBall;
    }


    /**
     * @param args input from user
     */
    public static void main(final String[] args) {
        String[] stringArray = new String[args.length];
        for (int index = 0; index < stringArray.length; index += 1) {
            stringArray[index] = args[index];
        }

        // now we have array of integers if there was no Exception.
        Point startPoint = new Point(0, 0);
        MultipleBouncingBallsAnimation m = new MultipleBouncingBallsAnimation(
                200, 200);
        Ball[] fullArrayBall = m.gettingBallFullArray(stringArray,
                m.screenWidth, m.screenLength, new Point(0, 0));
        BouncingBallAnimation b1 = new BouncingBallAnimation();
        GUI gui = new GUI("title", (int) m.screenWidth, (int) m.screenLength);
        b1.runBallAnimation(fullArrayBall, gui, m.screenWidth, m.screenLength,
                startPoint);

    }
}