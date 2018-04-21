import java.awt.Color;

import biuoop.DrawSurface;

/**
 * This program is a ScoreIndicator object.
 * which will be in charge of displaying the current score
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-24
 */
public class ScoreIndicator implements Sprite {
    /**
     * The ScoreIndicator will hold a reference to the scores counter.
     */
    private Counter scoreCounter;
    /**
     * Rectangle place for the background of the score on the screen.
     */
    private Rectangle rec;

    /**
     * Constructor of ScoreIndicator.
     *
     * @param score The counter of the score.
     */
    public ScoreIndicator(Counter score) {
        this.scoreCounter = score;
        this.rec = new Rectangle(new Point(0, 0), 800, 20);
    }

    @Override
    public void drawOn(DrawSurface d) {
        d.setColor(Color.WHITE);
        d.fillRectangle((int) this.rec.getUpperLeft().getX(), (int) this.rec.getUpperLeft().getY(),
                (int) this.rec.getWidth(), (int) this.rec.getHeight());
        d.setColor(Color.black);
        d.drawText((int) (50 + this.rec.getUpperLeft().getX() + this.rec.getWidth() / 3),
                (int) (this.rec.getUpperLeft().getY() + this.rec.getHeight()), "Score: " + this.scoreCounter
                        .getValue(), 18);

    }

    @Override
    public void timePassed() {
        // TODO Auto-generated method stub
    }

    @Override
    public void addToGame(GameLevel g) {
        // ScoreIndicator is a sprite interface, therefore:
        g.addSprite(this);
    }

}
