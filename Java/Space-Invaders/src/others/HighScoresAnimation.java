import java.awt.Color;
import java.util.List;

import biuoop.DrawSurface;
import biuoop.KeyboardSensor;

/**
 * HighScoresAnimation class to show the score table to the user of the game.
 * Each score has the name of the player & his new score.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-07
 */
public class HighScoresAnimation implements Animation {
    private String endKey;
    private HighScoresTable scoresTable;
    private KeyboardSensor keyboard;

    /**
     * Constructor function to HighScoresAnimation.
     *
     * @param scores      -the table of the scores (has names - scores)
     * @param keyToEnd    - It will display the scores in the high-scores table, until a
     *                    specified key is pressed.
     * @param newKeyboard - the keyboard to help us checking if the end key was pressed.
     */
    public HighScoresAnimation(HighScoresTable scores, String keyToEnd, KeyboardSensor newKeyboard) {
        this.scoresTable = scores;
        this.endKey = keyToEnd;
        this.keyboard = newKeyboard;
    }

    @Override
    public void doOneFrame(DrawSurface d, double dt) {
        // TODO Auto-generated method stub
        List<ScoreInfo> scoresList = this.scoresTable.getHighScores();
        // draw background rectangle.
        d.setColor(Color.decode("#ffff4d"));
        d.fillRectangle(0, 0, 800, 600);
        d.setColor(Color.RED);
        d.drawText(70, 60, "HighScores:", 45);
        d.setColor(Color.GREEN);
        d.drawText(100 + 130, 130, "Player Name:", 25);
        d.setColor(Color.GREEN);
        d.drawText(100 + 380, 130, "Score:", 25);
        d.setColor(Color.BLACK);
        d.drawLine(100 + 120, 130, 100 + 460, 130);
        for (int i = 0; i < scoresList.size(); i++) {
            d.setColor(Color.BLUE);
            d.drawText(100 + 170, 160 + 40 * i,
                    scoresList.get(i).getName(), 23);
            d.setColor(Color.decode("#0000ff"));
            d.drawText(100 + 400, 160 + 40 * i,
                    Integer.toString(scoresList.get(i).getScore()), 23);
        }
        d.setColor(Color.RED);
        d.drawText(250, 570,
                "Press space to continue", 30);
    }

    @Override
    public boolean shouldStop() {
        return this.keyboard.isPressed(this.endKey);
    }

}
