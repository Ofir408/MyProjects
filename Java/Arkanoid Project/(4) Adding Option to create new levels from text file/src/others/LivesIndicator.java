import java.awt.Color;

import biuoop.DrawSurface;

/**
 * This program is a LivesIndicator object.
 * which will be in charge of displaying the current score
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-24
 */
public class LivesIndicator implements Sprite {
    /**
     * Counter of Lives that the player still has.
     */
    private Counter numberOfLives;
    /**
     * Rectangle place for the background of the score on the screen.
     */
    private Rectangle rec;


    /**
     * Constructor of LivesIndicator.
     *
     * @param newNumberOfLives - Counter of number of lives.
     */
    public LivesIndicator(Counter newNumberOfLives) {
        this.numberOfLives = newNumberOfLives;
        this.rec = new Rectangle(new Point(0, 0), 800, 20); // need to change when 800 x 600
    }

    @Override
    public void drawOn(DrawSurface d) {
        d.setColor(Color.WHITE);
        // there is no needed to fill the rectangle because he already exists (when we
        // drawn the Score. So we just need to draw the Lives number in this rec.

        d.setColor(Color.black);
        d.drawText((int) (50 + this.rec.getUpperLeft().getX() + this.rec.getWidth() / 12),
                (int) (this.rec.getUpperLeft().getY() + this.rec.getHeight()), "Lives: "
                        + this.numberOfLives
                        .getValue(), 18);
    }

    @Override
    public void timePassed(double dt) {

    }

    @Override
    public void addToGame(GameLevel g) {
        // LivesIndicator is a Sprite interface, therefore:
        g.addSprite(this);
    }

    /**
     * @return LivesCounter ( a Counter ).
     */
    public Counter getLivesCounter() {
        return this.numberOfLives;
    }

}
