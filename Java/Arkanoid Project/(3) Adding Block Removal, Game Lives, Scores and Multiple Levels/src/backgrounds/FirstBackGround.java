import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import biuoop.DrawSurface;

/**
 * FirstBackGround Class - implements Sprite.
 * The first background in our game - "Direct-hit level"
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-27
 */
public class FirstBackGround implements Sprite {

    @Override
    public void drawOn(DrawSurface d) {


        Rectangle rec1 = new Rectangle(new Point(0, 800), 800, 800);  // need to check it
        Block b = new Block(rec1, 1, Color.BLACK);
        b.drawOn(d, Color.BLACK);

        // draw the background of "Direct-hit" level.
        // Let's start draw the circles

        int deltaX = 30;
        d.setColor(Color.BLUE);
        d.drawCircle(100 + 300, 140, 30);
        d.drawCircle(100 + 300, 140, 30 + deltaX);
        d.drawCircle(100 + 300, 140, 30 + 2 * deltaX);
        // d.drawLine(290, 110, 410, 110);
        //  d.drawLine(400, 50, 400, 220);
        d.drawLine(200 + 100, 135, 400 + 100, 135);
        d.drawLine(300 + 100, 20, 300 + 100, 240);

    }

    @Override
    public void timePassed() {
    }

    @Override
    public void addToGame(GameLevel g) {
        g.addSprite(this);
    }

    /**
     *
     * @return BlockSList.
     */
    public List<Block> getBlockList() {
        List<Block> blocksList = new ArrayList<>();

        Rectangle rec1 = new Rectangle(new Point(0, 0), 0, 0);  // need to check it
        Block b = new Block(rec1, 1, Color.BLACK);
        blocksList.add(b);

        Color c = Color.GRAY;
        Block[] slides = {new Block(new Rectangle(
                new Point(0, 40), 800, 20), 1, c),
                new Block(new Rectangle(new Point(
                        0, 600), 20, 600),
                        1, c),
                new Block(new Rectangle(new Point(
                        780, 600), 20, 600), 1, c),
                new Block(new Rectangle(new Point(
                        0, 620), 600, 20), 1, c)}; // the lowest block.


        for (int i = 0; i < slides.length; i += 1) {
            blocksList.add(slides[i]);
        }


        return blocksList;
    }
}
