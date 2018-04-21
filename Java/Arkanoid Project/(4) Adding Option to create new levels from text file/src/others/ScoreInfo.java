import java.io.Serializable;

/**
 * ScoreInfo class to handle with each score.
 * has the info about this new score -
 * the name of the player & his new score.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-06
 */
public class ScoreInfo implements Serializable {
    /**
     *
     */
    private static final long serialVersionUID = 1L;
    private String name;
    private int score;

    /**
     * @param playerName - the name of the player who did the new score.
     * @param newScore   - the new score.
     */
    public ScoreInfo(String playerName, int newScore) {
        this.name = playerName;
        this.score = newScore;
    }

    /**
     * @return the player who did this score.
     */
    public String getName() {
        return this.name;
    }

    /**
     * @return the new score.
     */
    public int getScore() {
        return this.score;
    }


}
