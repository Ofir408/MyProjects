
/**
 * This interface is a HitNotifier.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-22
 */
public interface HitNotifier {

    /**
     * Add hl as a listener to hit events.
     *
     * @param hl - HitListener interface.
     */
    void addHitListener(HitListener hl);

    /**
     * Remove hl from the list of listeners to hit events.
     *
     * @param hl - HitListener interface.
     */
    void removeHitListener(HitListener hl);
}
