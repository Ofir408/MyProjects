import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.Reader;
import java.util.ArrayList;
import java.util.List;

import biuoop.GUI;

/**
 * Ass5Game class.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-02
 */
public class Ass5Game {

    /**
     * @param args levels order numbers.
     *             run the game.
     */
    public static void main(final String[] args) {

        Ass5Game gameFive = new Ass5Game();
        String[] stringArray = new String[args.length];
        for (int index = 0; index < stringArray.length; index += 1) {
            stringArray[index] = args[index];
        }
        List<LevelInformation> finalLevelOrder = new ArrayList<>();
        finalLevelOrder.add(new GameLevel1());

        int numberFramesInSecond = 60;
        GUI gui = new GUI("Arkanoid", 800, 600);
        AnimationRunner runner = new AnimationRunner(gui, numberFramesInSecond);

        GameFlow gameFlow = new GameFlow(runner, gui.getKeyboardSensor(), 5);
        gameFlow.runLevels();

    }

    // helper functions

    /**
     * @param inputArray - input array from the user, that we check in this function.
     * @return new list of integers that contains level numbers according their order.
     */
    public List<Integer> convertJustToNumbers(String[] inputArray) {
        int currentNumber;

        List<Integer> numbersList = new ArrayList<Integer>();

        for (int index = 0; index < inputArray.length; index += 1) {
            try {
                currentNumber = Integer.parseInt(inputArray[index]);
                numbersList.add(currentNumber);
            } catch (Exception e) {
                index = index + 1;  // do nothing.
                index = index - 1;  // do nothing.
                // it is not a number, therefore we will not contain it in
                // new list to return;
            }
        }
        return numbersList;
    }

    /**
     * @param numbersList - numbers list from the function convertJustToNumbers.
     * @return get the list of numbers and return their order, if empty - return
     * default order of the levels.
     */
    public List<LevelInformation> getLevelsOrder(List<Integer> numbersList) {
        List<LevelInformation> levels = new ArrayList<>();
        List<Integer> legalNumbersList = new ArrayList<>();
        for (Integer currentNumb : numbersList) {
            if (currentNumb.intValue() > 0 && currentNumb.intValue() < 5) {
                // it in the scope of 1 - 4.
                legalNumbersList.add(currentNumb);
            }
        }
        for (Integer currentNumb : legalNumbersList) {
            levels.add(this.getLevel(currentNumb.intValue()));
        }
        if (levels.isEmpty()) {
            return this.getDefualtLevels();
        }
        return levels;
    }

    /**
     * This function created to make the code more elegant.
     *
     * @param numberOfLevel - the number of the level.
     * @return LevelInformation - the current level.
     */
    public LevelInformation getLevel(int numberOfLevel) {
        if (numberOfLevel == 1) {
            return new GameLevel1();
        }
        if (numberOfLevel == 2) {
            return new GameLevel2();
        }
        if (numberOfLevel == 3) {
            return new GameLevel3();
        } else {
            return new GameLevel4();
        }
    }

    /**
     * @return List<LevelInformation> - Default order Levels - 1, 2, 3, 4.
     */
    public List<LevelInformation> getDefualtLevels() {
        List<LevelInformation> levels = new ArrayList<>();
        Reader fileReader;

        try {
            Reader fileReader2 = new FileReader("C:/Users/user/workspace/ass2/resources/level_sets.txt");

            List<LevelInformation> m = SetsReaderOfLevels.
                    getLevelsOfSymbol("e", fileReader2); // just for checking, need to remove it later...
            System.out.println("My list is: " + m.size());
            levels.addAll(m);

            levels.add(new GameLevel1());
            levels.add(new GameLevel2());
            levels.add(new GameLevel3());
            levels.add(new GameLevel4());
            System.out.println("Added List<LevelInformation> :) ");
            return levels;
        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        //levels.addAll(lv);
        levels.add(new GameLevel1());
        levels.add(new GameLevel2());
        levels.add(new GameLevel3());
        levels.add(new GameLevel4());
        return levels;
    }

}
