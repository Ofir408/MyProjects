//package secondass;

import java.awt.Color;

import biuoop.DrawSurface;
import biuoop.GUI;
import biuoop.Sleeper;

/**
 * class BouncingBallAnimation.
 */
public class BouncingBallAnimation {

    /**
     * Function name: runBallAnimation.
     *
     * @param ballsArray   - array of balls that will move on the space.
     * @param gui          - helps us to draw.
     * @param screenWidth  - the width of the screen.
     * @param screenLength - the length of the screen.
     * @param startPoint   - the start point (left down) of the space
     *                     to draw the balls into him.
     */
    final void runBallAnimation(final Ball[] ballsArray, final GUI gui,
                                final double screenWidth,
                                final double screenLength,
                                final Point startPoint) {

        Sleeper sleeper = new Sleeper();

        while (true) {
            DrawSurface d = gui.getDrawSurface();
            for (int i = 0; i < ballsArray.length; i += 1) {

                ballsArray[i].moveOneStep(screenWidth, screenLength,
                        startPoint);


                // d.setColor(Color.BLUE);
                ballsArray[i].drawOn(d, Color.BLUE);


                sleeper.sleepFor(50);  // wait for 50 milliseconds.
            }
            gui.show(d);
        }
    }

    /**
     * @param args input from user
     */
    public static void main(final String[] args) {

        /*GUI gui = new GUI("title", 200, 200);
        //DrawSurface d = gui.getDrawSurface();
        Ball[] ballsArray = new Ball[2];
        BouncingBallAnimation b1 = new BouncingBallAnimation();
        Ball ball = new Ball(0, 0, 30, java.awt.Color.BLACK);
        Ball secondball = new Ball(20, 20, 30, java.awt.Color.BLUE);
        secondball.setVelocity(4, 3);
        ballsArray[0] = ball;
        ballsArray[1] = secondball;
        ball.setVelocity(4, 4);
        Point startPoint = new Point(0, 0);
        b1.runBallAnimation(ballsArray, gui, 200, 200, startPoint);
        //gui.show(d); */

        GUI gui = new GUI("title", 200, 200);
        Sleeper sleeper = new Sleeper();
        Ball ball = new Ball(0, 0, 30, java.awt.Color.BLACK);
        ball.setVelocity(2, 2);
        while (true) {
           ball.moveOneStep(200, 200, new Point(0, 0));
           DrawSurface d = gui.getDrawSurface();
           ball.drawOn(d, Color.BLACK);
           gui.show(d);
           sleeper.sleepFor(50);  // wait for 50 milliseconds.
        }

    }


}


