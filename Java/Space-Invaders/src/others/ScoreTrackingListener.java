
/**
 * This program is a ScoreTrackingListener object.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-24
 */
public class ScoreTrackingListener implements HitListener {
    /**
     * Counter object to count the current score that the player has.
     */
    private Counter currentScore;

    /**
     * @param scoreCounter - the Counter object for counting.
     */
    public ScoreTrackingListener(Counter scoreCounter) {
        this.currentScore = scoreCounter;
    }

    /**
     * @return get Counter object of currentScore.
     */
    public Counter getScoreCounter() {
        return this.currentScore;
    }

    @Override
    public void hitEvent(Block beingHit, Ball hitter) {
        if (beingHit.getHitPointsNumber() == 0) {
            this.currentScore.increase(100);
        }
    }

}
