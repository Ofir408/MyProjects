//package threeass;

import java.util.ArrayList;
import java.util.List;

import biuoop.DrawSurface;

/**
 * This program is a SpriteCollection class.
 * SpriteCollection will hold a collection of sprites.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-03
 */
public class SpriteCollection {
    /**
     * list of Sprite interfaces.
     */
    private List<Sprite> spriteList = new ArrayList<Sprite>();

    /**
     * add the given Sprite to the collection of spirits.
     *
     * @param s - Sprite object that we will add him to the collection.
     */
    public void addSprite(final Sprite s) {
        this.spriteList.add(s);
    }

    /**
     * Remove the given Sprite to the collection of spirits.
     *
     * @param s - Sprite object that we will remove him from the collection.
     */
    public void removeSprite(final Sprite s) {
        this.spriteList.remove(s);
    }

    /**
     * call timePassed() function on all sprites.
     * @param dt - the amount of seconds passed since the last call.
     */
    public void notifyAllTimePassed(double dt) {

        // You probably want to do the same when
        // iterating over the elements of the SpriteCollection
        List<Sprite> spriteCollection = new ArrayList<Sprite>(this.spriteList);

        // move on all sprites in out collection
        Sprite currentSprite;
        for (int i = 0; i < spriteCollection.size(); i += 1) {
            currentSprite = spriteCollection.get(i);
            currentSprite.timePassed(dt);
        }
    }

    /**
     * call drawOn(d) on all sprites.
     *
     * @param d - DrawSurface that helps us draw the sprites.
     */
    public void drawAllOn(final DrawSurface d) {
        // move on all sprites in our collection
        List<Sprite> spriteCollection = new ArrayList<Sprite>(this.spriteList);
        Sprite currentSprite;
        for (int i = 0; i < spriteCollection.size(); i += 1) {
            currentSprite = spriteCollection.get(i);
            currentSprite.drawOn(d);
        }
    }

    /**
     * @return get the collection of the sprites.
     */
    public List<Sprite> getSpriteCollection() {
        return this.spriteList;
    }


}
