import java.awt.Color;

import biuoop.DrawSurface;
import biuoop.KeyboardSensor;

/**
 * YouLose class.
 * calls when the player lose in the game, he has no more lifes.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-28
 */
public class YouLose implements Animation {
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
    public YouLose(KeyboardSensor k, int finalScore) {
        this.keyboard = k;
        this.stop = false;
        this.score = finalScore;
    }

    @Override
    public void doOneFrame(DrawSurface d) {
        d.setColor(Color.BLUE);
        d.drawText(200, 200, "Game Over, You Lose...", 35);
        d.setColor(Color.CYAN);
        d.drawText(280, 280, "Your score is: " + this.score, 35);
        d.setColor(Color.RED);
        d.drawText(180, 360, " - Success consists of going from failure to failure ", 25);
        d.drawText(180, 420, " Without loss of enthusiasm - Winston Churchill - ", 25);
        d.setColor(Color.orange);
        d.drawText(280, d.getHeight() / 5 + 350, "Press space to exit", 28);
        if (this.keyboard.isPressed(KeyboardSensor.SPACE_KEY)) {
            this.stop = true;
        }
    }

    @Override
    public boolean shouldStop() {
        return this.stop;
    }
}
