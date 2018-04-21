import java.util.List;

import biuoop.KeyboardSensor;

/**
 * This program is a GameFlow class.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-01
 */
public class GameFlow {

    private KeyboardSensor keyboardSensor;
    private AnimationRunner animationRunner;
    private int gameWidth;
    private int gameHeight;
    private int numberOfLives;
    private List<LevelInformation> levels;

    /**
     *
     * @param levelInfo - LevelInformation
     * @param r - AnimationRunner
     * @param k - KeyboardSensor
     * @param numberOfLives - number Of Lives.
     */
    public GameFlow(LevelInformation levelInfo, AnimationRunner r,
                    KeyboardSensor k, int numberOfLives) {
        animationRunner = r;
        keyboardSensor = k;
        this.numberOfLives = numberOfLives;
        this.gameWidth = 800;
        this.gameHeight = 600;
    }

    /**
     *
     * @param runner - AnimationRunner
     * @param ks - KeyboardSensor.
     * @param numberOfLives - number Of Lives.
     */
    public GameFlow(AnimationRunner runner, KeyboardSensor ks, int numberOfLives) {
        animationRunner = runner;
        keyboardSensor = ks;
        this.gameWidth = 800;
        this.gameHeight = 600;
        this.numberOfLives = numberOfLives;
    }

    /**
     *
     * @param levelsToRun - list of levels to run, each implements LevelInformation.
     */
    public void runLevels(List<LevelInformation> levelsToRun) {
        // ...

        Counter scoreCounter = new Counter(0);
        Counter lifeCounter = new Counter(numberOfLives);
        // gui = new GUI(title, width, height) // initialized in the Main

        int lastLevel = levelsToRun.size();
        int counterLevelPass = 0; // how much level was passed.
        for (LevelInformation levelInfo : levelsToRun) {

            GameLevel level = new GameLevel(levelInfo, this.animationRunner.getGui().getKeyboardSensor(),
                    animationRunner, lifeCounter, scoreCounter);


            level.initialize();
            //level has more blocks and player has more lives
            while (level.getNumberOfLivesCounter().getValue() > 0
                    && level.getBlocksCounter().getValue() > 0) {
                level.playOneTurn();
            }
            counterLevelPass += 1; // more level was passed positive.
            if (counterLevelPass == lastLevel) {
                // finished good all the levels - You win in the game! :)
                this.animationRunner.run(new YouWin(keyboardSensor, scoreCounter.getValue()));
                this.animationRunner.getGui().close();
            }
        }
    }
}
