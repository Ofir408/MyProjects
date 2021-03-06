//package threeass;

import biuoop.DrawSurface;

/**
 * This program is a Sprite interface.
 * Sprites can be drawn on the screen, and can be notified
 * that time has passed (so that they know to change their
 * position / shape / appearance / etc). In our design,
 * all of the game objects (Ball, Block, Paddle, ...) are Sprites
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-03
 */

public interface Sprite {

    /**
     * draw the sprite to the screen.
     *
     * @param d - DrawSurface object.
     */
    void drawOn(DrawSurface d);

    /**
     * notify the sprite that time has passed.
     * @param dt - the amount of seconds passed since the last call.
     */
    void timePassed(double dt);

    /**
     * adding the sprite interface to the game g.
     *
     * @param g - the game that we will add the sprite to him.
     */
    void addToGame(GameLevel g);



}
