import biuoop.DrawSurface;

/**
 * This program is a Game class.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-03
 */
public interface Animation {

    /**
     * doOneFrame(DrawSurface) is in charge of the logic.
     *
     * @param d - DrawSurface.
     */
    void doOneFrame(DrawSurface d);

    /**
     * is in charge of stopping condition.
     *
     * @return true if the level should stop (because of no more balls,
     * or no more blocks). Otherwise, return false.
     */
    boolean shouldStop();


}
