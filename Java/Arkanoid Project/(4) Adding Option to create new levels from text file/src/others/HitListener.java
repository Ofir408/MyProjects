
/**
 * This program is an interface HitListener.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-22
 */

public interface HitListener {

    /**
     * This method is called whenever the beingHit object is hit.
     * The hitter parameter is the Ball that's doing the hitting.
     *
     * @param beingHit - the Block that hit.
     * @param hitter   - the ball that hit in the Block, it means
     *                 the Ball that's doing the hitting.
     */

    void hitEvent(Block beingHit, Ball hitter);

}
