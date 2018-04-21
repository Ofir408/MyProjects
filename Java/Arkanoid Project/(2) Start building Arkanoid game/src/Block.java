//package threeass;

import java.awt.Color;

import biuoop.DrawSurface;
//import secondass.Ball;
//import secondass.Point;
//import secondass.Velocity;

/**
 * That's Block class.
 * that represents block that thing can collide with him.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-04-27
 */

public class Block implements Collidable, Sprite {

    /**
     * the block is a rectangle object.
     */
    private Rectangle rec;
    /**
     * Each block should be initialized with a positive number of hit-points
     * number of hits in this block - initialized with the maximum
     * number of hits in the block, and then decrease in each more hit.
     * when the number of points is 0, it will stay 0)
     */
    private int hitPoints;
    /**
     * the color of this block.
     */
    private Color color;

    // functions

    /**
     * Constructor block object.
     *
     * @param r             - Rectangle object.
     * @param maxHitsNumber - initialized with the maximum
     *                      number of hits in the block.
     * @param newColor      - the color of this block.
     */
    public Block(final Rectangle r, final int maxHitsNumber,
                 final Color newColor) {
        this.rec = r;
        this.hitPoints = maxHitsNumber;
        this.color = newColor;
    }

    /**
     * @return the "collision shape" of the object.
     */
    @Override
    public Rectangle getCollisionRectangle() {
        return this.rec;
    }

    /**
     * @return the number of the hit points
     * (number of max hits - number of hits until now)
     * while decrease hitPoints in each more hit.
     * But when the number of points is 0, it will stay 0
     */
    public int getHitPointsNumber() {
        return this.hitPoints;
    }

    /**
     * @return the color of the current block
     */
    public Color getColor() {
        return this.color;
    }

    /**
     * this function calls when there is an hit.
     * decrease hitPoints because there was more hit.
     * But when the number of points is 0, it will stay 0,
     * so if it's possible - decrease. Otherwise - do nothing.
     */
    public void decrease() {
        if (this.hitPoints > 0) {
            // it's possible to decrease
            this.hitPoints -= 1;
        } else {
            // do nothing, just confirm it's stay 0.
            this.hitPoints = 0;
        }
    }

    @Override
    /**
     * This function is called when there is an hit.
     * @param collisionPoint - the collision Point between
     * the block and the other object
     * (ball).
     * @param currentVelocity - the velocity before the hit between them.
     * @return the new Velocity after ths hit.
     */
    public Velocity hit(final Point collisionPoint,
                        final Velocity currentVelocity, final Ball ball) {
        // there was an hit so calls to decrease function
        this.decrease();

        double xVelocity = currentVelocity.getDx();
        double yVelocity = currentVelocity.getDy();

        if (this.hitX(collisionPoint)) {
            if (this.hitY(collisionPoint)) {
                // change both
                return new Velocity(-1 * xVelocity,
                        -1 * yVelocity);
            } else {
                // hit x but y no, therefore change just x
                return new Velocity(-1 * xVelocity, yVelocity);
            }
        }
        // x no according first checking so change just y
        return new Velocity(xVelocity, -1 * yVelocity);
    } // */
    //}

    /**
     * this function checks if there is intersection with X.
     *
     * @param collisionPoint - the collision Point between
     *                       the block and the ball.
     * @return true if the collision Point hit X
     */
    public final boolean hitX(final Point collisionPoint) {
        if (this.rec.getUpperLeft().getX() == collisionPoint.getX()
                || this.rec.getUpperLeft().getX()
                + this.rec.getWidth()
                == collisionPoint.getX()) {
            if (collisionPoint.getY()
                    <= this.rec.getUpperLeft().getY()
                    && collisionPoint.getY()
                    >= this.rec.getUpperLeft().getY()
                    - this.rec.getHeight()) {
                return true;
            }
        }
        return false;
    }

    /**
     * this function checks if there is intersection with Y.
     *
     * @param collisionPoint - the collision Point between
     *                       the block and the ball.
     * @return true if the collision Point hit Y.
     */
    public final boolean hitY(final Point collisionPoint) {
        if (this.rec.getUpperLeft().getY() == collisionPoint.getY()
                || this.rec.getUpperLeft().getY()
                - this.rec.getHeight()
                == collisionPoint.getY()) {
            if (collisionPoint.getX()
                    >= this.rec.getUpperLeft().getX()
                    && collisionPoint.getX()
                    <= this.rec.getUpperLeft().getX()
                    + this.rec.getWidth()) {
                return true;
            }
        }
        return false;
    }

    /**
     * The function draw the current block on the given DrawSurface.
     * void function.
     *
     * @param surface - input DrawSurface that helps us to draw the block
     *                object about him.
     */
    // draw the ball on the given DrawSurface
    @Override
    public void drawOn(final DrawSurface surface) {
        surface.setColor(this.color);
        surface.fillRectangle((int) this.rec.getUpperLeft().getX(),
                (int) this.rec.getUpperLeft().getY()
                        - (int) this.rec.getHeight(),
                (int) this.rec.getWidth(), (int) this.rec.getHeight());
        this.drawNumberHits(surface, Color.WHITE);
    }


    /**
     * The function draw the current block on the given DrawSurface.
     * void function.
     *
     * @param surface  - input DrawSurface that helps us to draw the block
     *                 object about him.
     * @param newColor - new color of the circle to draw.
     */
    // draw the ball on the given DrawSurface
    public void drawOn(final DrawSurface surface,
                       final Color newColor) {
        surface.setColor(newColor);
        surface.fillRectangle((int) this.rec.getUpperLeft().getX(),
                (int) this.rec.getUpperLeft().getY()
                        - (int) this.rec.getHeight(),
                (int) this.rec.getWidth(), (int) this.rec.getHeight());
        this.drawNumberHits(surface, Color.WHITE);
    }
    /**
     * The function draw the current block on the given DrawSurface.
     * void function.
     *
     * @param surface  - input DrawSurface that helps us to draw the block
     *                 object about him.
     * @param newColor - new color of the circle to draw.
     */
    // draw the ball on the given DrawSurface
    public void drawBackground(final DrawSurface surface,
                       final Color newColor) {
        surface.setColor(newColor);
        surface.fillRectangle((int) this.rec.getUpperLeft().getX(),
                (int) this.rec.getUpperLeft().getY()
                        - (int) this.rec.getHeight(),
                (int) this.rec.getWidth(), (int) this.rec.getHeight());
         // this.drawNumberHits(surface, Color.WHITE);
    }

    /**
     * The hit point will be indicated visually
     * on the block (as a white number / letter in the middle of the block).
     * block with 0 hit-points should have a 'X' displayed on it
     *
     * @param surface  - input DrawSurface that helps us to draw the block
     *                 object about him.
     * @param newColor - new color of the circle to draw.
     */
    public void drawNumberHits(final DrawSurface surface,
                               final Color newColor) {
        surface.setColor(Color.WHITE);
        // check where is the middle of the block,
        // because we want to draw there.
        double xPlace = this.rec.getUpperLeft().getX() + this.rec.getWidth()
                / 2.00;
        double yPlace = this.rec.getUpperLeft().getY() - this.rec.getHeight()
                / 2.00;
        if (this.hitPoints < 1) {
            // draw "X"
            surface.drawText((int) xPlace, (int) yPlace, "X", 15);
        } else {
            // more than 1 - so the number is displayed on the middle
            // of the current block.
            surface.drawText((int) xPlace, (int) yPlace,
                    String.valueOf(this.hitPoints), 15);
        }
    }

    @Override
    public void timePassed() {
        // need to fill it in the continue of the assignment.
    }

    @Override
    public final void addToGame(final Game g) {
        //  Block is both a sprite and a collidable
        //  when we add a Block we need to call both
        //  addSprite and addCollidable.
        g.addCollidable(this);
        g.addSprite(this);

    }

}
