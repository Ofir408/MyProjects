
/**
 * a BlockRemover is in charge of removing blocks from the game, as well as
 * keeping count of the number of blocks that remain.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-23
 */

public class BlockRemover implements HitListener {
    /**
     * our Game object.
     */
    private GameLevel game;
    /**
     * keeping count of the number of blocks that remain, Counter object.
     */
    private Counter remainingBlocks;

    /**
     * Constructor function of BlockRemover object.
     *
     * @param newGame       - our Game object.
     * @param removedBlocks - counter of how much blocks still remain.
     */
    public BlockRemover(GameLevel newGame, Counter removedBlocks) {
        this.game = newGame;
        this.remainingBlocks = removedBlocks;
    }

    /**
     * Blocks that are hit and reach 0 hit-points should be removed from the
     * game. Remember to remove this listener from the block that is being
     * removed from the game.
     *
     * @param beingHit - the block that hit and maybe we will remove him from the
     *                 current game (if it reach 0 hit-points).
     * @param hitter   - the ball that hit this Block.
     */
    @Override
    public void hitEvent(Block beingHit, Ball hitter) {


        System.out.println("hits number is: " + beingHit.getHitPointsNumber());
        System.out.println("Am I a shield? " + beingHit.getIfShield());
        System.out.println("This ball doesnt from Alien? " + !hitter.getIsBallFromAlien());

        //if (beingHit.getHitPointsNumber() == 0 && (!hitter.getIsBallFromAlien() || beingHit.getIfShield())) {
        //if ((beingHit.getIfShield() || !hitter.getIsBallFromAlien())) {
        // reached 0 hit-points so we should remove the listener from the
        // game.
        beingHit.removeHitListener(this);
        beingHit.removeFromGame(this.game);
        // check if need to delete //
        beingHit.setRecToHitted();
        // remove the ball
        hitter.setCenterPoint(new Point(-30, -30)); // out of screen
        if (!beingHit.getIfShield()) {
            this.remainingBlocks.decreaseOne();
        }
        // }
    }

    /**
     * @return Counter object of Remaining Block in
     */
    public Counter getCounterRemainBlocks() {
        return this.remainingBlocks;
    }
}
