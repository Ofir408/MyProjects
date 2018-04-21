import java.awt.Color;

import biuoop.DrawSurface;

/**
 * This program is a NameLevelIndicator class.
 * this class helps us to draw the name of the current level,
 * on the screen of the game.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-27
 */
public class NameLevelIndicator implements Sprite {
    /**
     * Counter of Lives that the player still has.
     */
    private String levelName;
    /**
     * Rectangle place for the background of the score on the screen.
     */
    private Rectangle rec;

    /**
     *
     * @param nameOfLevel - the name of level to show.
     */
    public NameLevelIndicator(String nameOfLevel) {
        this.levelName = nameOfLevel;
        this.rec = new Rectangle(new Point(0, 0), 800, 20); // need to change when 800 x 600.
    }

    @Override
    public void drawOn(DrawSurface d) {
        d.setColor(Color.black);
        d.drawText((int) (450 + this.rec.getUpperLeft().getX() + this.rec.getWidth() / 12),
                (int) (this.rec.getUpperLeft().getY() + this.rec.getHeight()), "Level Name: "
                        + this.levelName, 18);

    }

    @Override
    public void timePassed() {
    }

    @Override
    public void addToGame(GameLevel g) {
        g.addSprite(this);
    }
}
