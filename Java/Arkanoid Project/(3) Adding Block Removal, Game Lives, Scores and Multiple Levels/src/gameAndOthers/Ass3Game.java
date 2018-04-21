//package threeass;

import biuoop.GUI;

/**
 * This program is a Ass3Game.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-06
 */

public class Ass3Game {

    /**
     * run the game.
     *
     * @param args - none.
     */
    public static void main(final String[] args) {
        GUI gui = new GUI("Arkanoid", 600, 600);
        GameLevel game = new GameLevel();
        game.run();
    }
}
