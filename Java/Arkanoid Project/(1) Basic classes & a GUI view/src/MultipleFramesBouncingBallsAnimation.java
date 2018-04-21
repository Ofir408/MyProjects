//package secondass;

import java.awt.Color;

import biuoop.DrawSurface;
import biuoop.GUI;
import biuoop.Sleeper;

/**
 * class MultipleFramesBouncingBallsAnimation.
 *
 * @author Ofir Ben Shoham
 * @version 1.1
 * @since 2017-04-08
 */
public class MultipleFramesBouncingBallsAnimation {

    /**
     * function that draw two rectangles and then puts the input balls
     * in the rectangles and makes moves.
     *
     * @param ballsArray       - array of balls.
     * @param gui              - help to draw.
     * @param screenWidth      - the width of the screen.
     * @param screenLength     - the length of the screen.
     * @param firststartPoint  - the left down start point of the
     *                         first rectangle.
     * @param secondStartPoint - the left down start point of the
     *                         second rectangle.
     */
    public void drawRectangle(final Ball[] ballsArray, final GUI gui,
                              final double screenWidth,
                              final double screenLength,
                              final Point firststartPoint,
                              final Point secondStartPoint) {

        // draw first rectangle: a grey rectangle (50,50) to (500,500),
        Sleeper sleeper = new Sleeper();

        //double totalDx = ball.getSize(), totalDy = ball.getSize();
        while (true) {
            DrawSurface d = gui.getDrawSurface();
            d.setColor(Color.GRAY);
            d.fillRectangle(50, 50, 450, 450);
            //d.drawRectangle(50, 50, 450, 450);
            d.setColor(Color.YELLOW);
            d.fillRectangle(450, 450, 150, 150);
            for (int i = 0; i < ballsArray.length; i += 1) {


                //ballsArray[i].drawOn(d);

                if (i < ballsArray.length / 2) {
                    ballsArray[i].moveOneStep(450, 450,
                            firststartPoint);
                    ballsArray[i].drawOn(d, Color.RED);
                } else {
                    ballsArray[i].moveOneStep(150, 150,
                            secondStartPoint);
                    ballsArray[i].drawOn(d, Color.BLUE);
                }
                d.setColor(Color.BLUE);
                //ballsArray[i].drawOn(d, );
                sleeper.sleepFor(50);  // wait for 50 milliseconds.
            }
            gui.show(d);

        }
    }

    /**
     * @param args input from user
     */
    public static void main(final String[] args) {
        GUI gui = new GUI("title", 600, 600);
        MultipleFramesBouncingBallsAnimation mFrames =
                new MultipleFramesBouncingBallsAnimation();

        MultipleBouncingBallsAnimation mBouncingBall = new
                MultipleBouncingBallsAnimation(450, 450);

        String[] stringArray = new String[args.length];
        for (int index = 0; index < stringArray.length; index += 1) {
            stringArray[index] = args[index];
        }
        Ball[] fullArrayBall = mBouncingBall.gettingBallFullArray(stringArray,
                mBouncingBall.getScreenWidth(),
                mBouncingBall.getScreenLength(), new Point(50, 50));

        // change startPoint for the second array
        //(to space of the second rectangle)


        for (int i = fullArrayBall.length / 2; i < fullArrayBall.length; i++) {
            fullArrayBall[i].setCenterPoint(
                    mBouncingBall.generateRandomPoint(150,
                            150, fullArrayBall[i].getSize(),
                            new Point(450, 450)));
        }


        /*Ball[] ballsArray = new Ball[1];
        Ball ballF1 = new Ball(403, 390, 3, Color.blue);
        ballF1.setVelocity(5, 5);
        Ball ballF2 = new Ball(0, 30, 4, Color.green);
        ballF2.setVelocity(4, 10);
        Ball ballF3 = new Ball(50, 60, 4, Color.red);
        ballF3.setVelocity(5, 6);
        Ball ballF4 = new Ball(0, 0, 4, Color.yellow);
        ballF4.setVelocity(2, 2);
        //ballsArray[0] = ballF1;
        ballsArray[0] = ballF3; */
        // ballsArray[1] = ballF2; ballsArray[3] = ballF4;
        //gui.show(d);
        //DrawSurface d = gui.getDrawSurface();
        mFrames.drawRectangle(fullArrayBall, gui, 550, 550, new Point(50, 50),
                new Point(450, 450));


    }

}
