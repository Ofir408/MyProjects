import java.awt.Color;

import biuoop.DrawSurface;
import biuoop.KeyboardSensor;

/**
 * This program is YouWin class.
 * This class creates when the player win the game - finish all the blocks
 * in all the levels.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-03
 */
public class YouWin implements Animation {
    /**
     * in order to use in "P" key to pause the game.
     */
    private KeyboardSensor keyboard;

    /**
     * stop is true if we want to stop the game. Otherwise, false.
     */

    private boolean stop;
    /**
     * final score in the game.
     */
    private int score;

    /**
     * @param finalScore - final score in the game to show.
     * @param k          - KeyboardSensor.
     */
    public YouWin(KeyboardSensor k, int finalScore) {
        this.keyboard = k;
        this.stop = false;
        this.score = finalScore;
    }

    @Override
    public void doOneFrame(DrawSurface d) {
        d.setColor(Color.BLUE);
        d.drawText(280, 200, "Wow, You Win!", 35);
        d.setColor(Color.CYAN);
        d.drawText(280, 280, "Your score is: " + this.score, 35);
        d.setColor(Color.GREEN);
        d.drawText(280, d.getHeight() / 5 + 250, "Press space to continue", 28);
        if (this.keyboard.isPressed(KeyboardSensor.SPACE_KEY)) {
            this.stop = true;
        }
    }

    @Override
    public boolean shouldStop() {
        return this.stop;
    }

}
