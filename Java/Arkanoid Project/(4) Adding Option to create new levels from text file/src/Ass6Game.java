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

        try {
            String levelSetsPath;
            if (args.length > 0) {
                levelSetsPath = args[0];
            } else {
                levelSetsPath = "level_sets.txt";
            }

            int numberFramesInSecond = 60;
            GUI gui = new GUI("Arkanoid", 800, 600);
            AnimationRunner runner = new AnimationRunner(gui, numberFramesInSecond);

            GameFlow gameFlow = new GameFlow(runner, gui.getKeyboardSensor(), 7);
            gameFlow.setPath(levelSetsPath);

            gameFlow.runLevels();
        } catch (Exception e) {
            System.out.println("Exception was caught in the main :( ");
        }


        // gameFlow.runLevels(finalLevelOrder);

    }

}
