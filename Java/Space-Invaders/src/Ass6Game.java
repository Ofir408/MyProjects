import biuoop.GUI;

/**
 * Ass6Game class.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-21
 */
public class Ass6Game {
    /**
     * @param args - none.
     */
    public static void main(final String[] args) {

            int numberFramesInSecond = 60;
            GUI gui = new GUI("Space-invaders", 800, 600);
            AnimationRunner runner = new AnimationRunner(gui, numberFramesInSecond);

            GameFlow gameFlow = new GameFlow(runner, gui.getKeyboardSensor(), 3);

            gameFlow.runLevels();

    }

}
