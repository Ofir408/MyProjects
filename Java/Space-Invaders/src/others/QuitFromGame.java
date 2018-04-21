
/**
 * This program has a Task to Quit from the game.
 * using System.exit().s
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-09
 */
public class QuitFromGame implements Task<Void> {


    @Override
    public Void run() {
        // exit from the game.
        System.exit(0);
        return null;
    }

}
