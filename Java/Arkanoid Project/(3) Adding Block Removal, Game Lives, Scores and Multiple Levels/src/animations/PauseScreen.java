import java.awt.Color;

import biuoop.DrawSurface;
import biuoop.KeyboardSensor;

/**
 * That's PauseScreen class.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-28
 */
public class PauseScreen implements Animation {

    /**
     * in order to use in "P" key to pause the game.
     */
    private KeyboardSensor keyboard;
    /**
     * stop is true if we want to stop the game. Otherwise, false.
     */

    private boolean stop;

    /**
     * @param k - KeyboardSensor.
     */
    public PauseScreen(KeyboardSensor k) {
        this.keyboard = k;
        this.stop = false;
    }

    @Override
    public void doOneFrame(DrawSurface d) {
        d.setColor(Color.GREEN);
        d.drawText(360, d.getHeight() / 2 - 200, "Paused", 32);
        d.setColor(Color.BLUE);
        d.drawText(100, d.getHeight() / 4 + 80, "You are amazing player ! Why did you stop playing?", 28);
        d.setColor(Color.RED);
        d.drawText(100, d.getHeight() / 5 + 150, "Maybe do you want to continue the game?", 28);
        d.setColor(Color.decode("#8A2BE2"));
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
