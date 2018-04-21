import java.awt.Color;

import biuoop.DrawSurface;
import biuoop.Sleeper;

/**
 * That's CountdownAnimation class.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-28
 */
public class CountdownAnimation implements Animation {

    /**
     * number of second for waiting.
     */
    private double numOfSeconds;
    /**
     * from which number start counting bottom.
     */
    private int countFrom;
    /**
     * Spirte collection to draw.
     */
    private SpriteCollection gameScreen;
    /**
     * true if should stop. Otherwise false.
     */
    private boolean shouldStop;

    /**
     * @param newNumOfSeconds    - number of seconds for each show.
     * @param numberCountingFrom - from which number start to count.
     * @param newGameScreen      - Spirte collection to draw.
     */

    public CountdownAnimation(double newNumOfSeconds,
                              int numberCountingFrom,
                              SpriteCollection newGameScreen) {
        this.numOfSeconds = newNumOfSeconds;
        this.countFrom = numberCountingFrom;
        this.gameScreen = newGameScreen;
    }

    /**
     * DecreaseOne for the counter.
     */
    public void decreaseOne() {
        this.countFrom -= 1;
    }

    /**
     * Does one frame of the countdown animation.
     *
     * @param d This is the drawsurface we draw on.
     */
    @Override
    public void doOneFrame(DrawSurface d) {
        this.gameScreen.drawAllOn(d);
        Sleeper sleeper = new Sleeper();
        double timePerNumber = 1000 * this.numOfSeconds / 2;
        long startTime = System.currentTimeMillis();

        d.setColor(Color.RED);
        if (this.countFrom > 0) {
            d.drawText(200, 100, "" + this.countFrom + "... ", 40);
            this.decreaseOne();
        } else if (this.countFrom == 0) {
            d.drawText(300, 300, "GO", 35);
            this.decreaseOne();
        } else {
            this.shouldStop = true;
        }

        long usedTime = System.currentTimeMillis() - startTime;
        long milliSecondLeftToSleep = (long) timePerNumber - usedTime;
        if (milliSecondLeftToSleep > 0) {
            sleeper.sleepFor(milliSecondLeftToSleep);
        }
    }

    /**
     * @return true if need to stop.
     */
    @Override
    public boolean shouldStop() {
        return this.shouldStop;
    }
}
