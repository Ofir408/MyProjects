
/**
 * This program is a BallRemover object.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-23
 */
public class BallRemover implements HitListener {

    /**
     * our Game object.
     */
    private GameLevel game;
    /**
     * keeping count of the number of blocks that remain, Counter object.
     */
    private Counter remainingBalls;

    /**
     * Constructor function of BlockRemover object.
     *
     * @param newGame     - our Game object.
     * @param remainBalls - counter of how much blocks still remain.
     */
    public BallRemover(GameLevel newGame, Counter remainBalls) {
        this.game = newGame;
        this.remainingBalls = remainBalls;
    }

    @Override
    public void hitEvent(Block beingHit, Ball hitter) {
        hitter.removeFromGame(game);
        this.remainingBalls.decreaseOne();
    }

    /**
     * @return Counter object of Remaining Balls in
     */
    public Counter getCounterRemainBalls() {
        return this.remainingBalls;
    }
}
