import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import biuoop.DialogManager;
import biuoop.GUI;
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
    private HighScoresTable highScoreTable;

    /**
     * @param levelInfo     - LevelInformation
     * @param r             - AnimationRunner
     * @param k             - KeyboardSensor
     * @param numberOfLives - number Of Lives.
     */
    public GameFlow(LevelInformation levelInfo, AnimationRunner r, KeyboardSensor k, int numberOfLives) {
        animationRunner = r;
        keyboardSensor = k;
        this.numberOfLives = numberOfLives;
        this.gameWidth = 800;
        this.gameHeight = 600;
    }

    /**
     * @param runner        - AnimationRunner
     * @param ks            - KeyboardSensor.
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
     * empty constructor.
     */
    public GameFlow() {
    }



    /**
     * run the levels of the game.
     */
    public void runLevels() {

        try {
            File highScoresFile = new File("highscores");
            this.highScoreTable = HighScoresTable.loadFromFile(highScoresFile);
            if (this.highScoreTable.getHighScores().isEmpty()) {
                this.highScoreTable = new HighScoresTable(10);
            }
            this.highScoreTable.save(highScoresFile);
        } catch (IOException io) {
            System.out.println("There is a problem to create initial file of highscores (10)");
        }


        Menu<Task<Void>> menu = new MenuAnimation<Task<Void>>("Arknoid", this.keyboardSensor, this.animationRunner);
        Menu<Task<Void>> subMenu = new MenuAnimation<>("subMenu", this.keyboardSensor, this.animationRunner);



            List<LevelInformation> levls = new ArrayList<>();
            levls.add(new GameLevel1().setName("Level " + levls.size()));

            //levls.get(levls.lastIndexOf(new GameLevel1().setName("Level " + levls.size())));
            subMenu.addSelection("b", "begin",
                    new EasyLevels(levls, this));


        // add sub menu
        menu.addSubMenu("", "", subMenu);

        menu.addSelection("s", "Start the game",
                new StartTheGameFromMenu(this.animationRunner, this.keyboardSensor, subMenu));

        menu.addSelection("h", "Show high score table", new ShowHiScoresTask(this.animationRunner,
                new HighScoresAnimation(this.highScoreTable, KeyboardSensor.SPACE_KEY, keyboardSensor)));
        menu.addSelection("q", "Quit", new QuitFromGame());

        // menu.


        while (true) {
            this.animationRunner.run(menu);
            Task<Void> result = menu.getStatus();
            result.run();
        }
    }

    /**
     * @param gui - GUI.
     * @return the name of the player who made the new score in the game. call
     * to this function just if its score above others scores in our
     * score table.
     */
    static String getNameFromUser(GUI gui) {
        DialogManager dialog = gui.getDialogManager();
        String name = dialog.showQuestionDialog("Name", "What is your name?", "");
        return name;
    }

    /**
     * @param s - Counter that has the final score of the player in this game.
     */
    private void checkIfBetterScore(Counter s) {
        if (this.highScoreTable.getRank(s.getValue()) < this.highScoreTable.size()) {
            String playerName = GameFlow.getNameFromUser(this.animationRunner.getGui());
            this.highScoreTable.add(new ScoreInfo(playerName, s.getValue()));
            try {
                File highScoreFile = new File("highscores");
                this.highScoreTable.save(highScoreFile);
            } catch (IOException io) {
                System.out.println("A problem to save the new score in our exist table scores (11)");
            }
        }
    }

    /**
     * handle when winning - check if better score.
     *
     * @param scoreCounter - counter of the score in the current game.
     * @param levelsToRun  - List<LevelInformation> to run.
     */
    public void handleWithWin(Counter scoreCounter, List<LevelInformation> levelsToRun) {
        // finished good all the levels - You win in the game! :)
        this.checkIfBetterScore(scoreCounter);
        this.animationRunner.run(new KeyPressStoppableAnimation(keyboardSensor, KeyboardSensor.SPACE_KEY,
                new YouWin(keyboardSensor, scoreCounter.getValue())));
        Animation a = new HighScoresAnimation(highScoreTable, KeyboardSensor.SPACE_KEY, keyboardSensor);
        this.animationRunner.run(new KeyPressStoppableAnimation(keyboardSensor, KeyboardSensor.SPACE_KEY, a));
        this.runLevels();
        this.animationRunner.getGui().close();
    }

    /**
     * @param levelsToRun - List<LevelInformation> for run.
     */
    public void runLevels2(List<LevelInformation> levelsToRun) {

        Counter scoreCounter = new Counter(0);
        Counter lifeCounter = new Counter(numberOfLives);
        int lastLevel = levelsToRun.size();
        int counterLevelPass = 0;

        for (LevelInformation levelInfo : levelsToRun) {
            File highScoresFile = new File("highscores");
            this.highScoreTable = HighScoresTable.loadFromFile(highScoresFile);
            if (this.highScoreTable == null) {
                System.out.println("There is a problem to create initial file of highscores (10)");
            }


            GameLevel level = new GameLevel(levelInfo, this.animationRunner.getGui().getKeyboardSensor(),
                    animationRunner, lifeCounter, scoreCounter);

            level.setHighScoreTable(this.highScoreTable);

            level.initialize();

            // level has more blocks and player has more lives

            while (level.getNumberOfLivesCounter().getValue() > 0
                    && level.getBlocksCounter().getValue() > 0) {
                level.playOneTurn();
                // reset data


                // check life's number
                if (lifeCounter.getValue() == 0) {
                    this.hadleWithLose(scoreCounter);
                    this.runLevels();
                }
            }

            counterLevelPass += 1; // more level was passed positive.
            if (lifeCounter.getValue() == -1) {
                this.hadleWithLose(scoreCounter);
                this.runLevels();
            }
            level.setLevelName();

            if (counterLevelPass == lastLevel && lifeCounter.getValue() > 0) {
                this.handleWithWin(scoreCounter, levelsToRun);
            }
        }
    }

    /**
     * @param counterScore - Counter with the score of the player who losed.
     */
    private void hadleWithLose(Counter counterScore) {
        this.checkIfBetterScore(counterScore);

        this.animationRunner.run(new KeyPressStoppableAnimation(
                this.keyboardSensor, KeyboardSensor.SPACE_KEY,
                new YouLose(this.keyboardSensor, counterScore.getValue())));

        this.animationRunner.run(new KeyPressStoppableAnimation(
                this.keyboardSensor, KeyboardSensor.SPACE_KEY,
                new HighScoresAnimation(this.highScoreTable,
                        KeyboardSensor.SPACE_KEY, this.keyboardSensor)));
    }
}

