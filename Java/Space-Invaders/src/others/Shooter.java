/**
 * Shooter sprite that who that implements him need to know how to shot balls
 * Who will implement the shooter interface?
 * the alien class & the paddle class.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-03
 */
public interface Shooter {

    /**
     * Shot new ball and add him to our game.
     *
     * @param gameLavel - to enter the new ball that was shot.
     */
    void shot(GameLevel gameLavel);

}
