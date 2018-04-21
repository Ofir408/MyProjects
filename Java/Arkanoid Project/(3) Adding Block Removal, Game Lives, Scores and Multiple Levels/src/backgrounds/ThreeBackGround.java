import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import biuoop.DrawSurface;

/**
 * ThreeBackGround Class - implements Sprite.
 * The three background in our game - "Green 3"
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-29
 */
public class ThreeBackGround implements Sprite {

    @Override
    public void drawOn(DrawSurface d) {
        this.drawRectangles(d);
        this.drawCircles(d);
    }

    @Override
    public void timePassed() {
    }

    @Override
    public void addToGame(GameLevel g) {
        g.addSprite(this);
    }

    /**
     * draw the Circles.
     *
     * @param d - DrawSurface.
     */
    private void drawCircles(DrawSurface d) {
        Point p1 = new Point(100, 250);
        int x = 4;
        d.setColor(Color.yellow);
        d.fillCircle((int) p1.getX(), (int) p1.getY(), 3 * x);
        d.setColor(Color.RED);
        d.fillCircle((int) p1.getX(), (int) p1.getY(), 2 * x);
        d.setColor(Color.WHITE);
        d.fillCircle((int) p1.getX(), (int) p1.getY(), x);
    }

    /**
     * draw the Rectangles.
     *
     * @param d - DrawSurface.
     */
    private void drawRectangles(DrawSurface d) {
        int number = 5;
        int i = 1;
        d.setColor(Color.decode("#2a8215"));
        d.fillRectangle(0, 0, 800, 600);
        d.setColor(Color.gray);
        d.fillRectangle(85, 410, 30, 50);
        d.setColor(Color.lightGray);
        d.fillRectangle(95, 250, 10, 160);
        d.setColor(Color.darkGray);
        d.fillRectangle(50, 450, 95, 160);
        d.setColor(Color.white);

        while (i < 6) {
            for (int secondCounter = 0; secondCounter < number; secondCounter += 1) {
                d.fillRectangle(57 + 17 * (i - 1), 490 + 30 * (secondCounter - 1),
                        10, 25);
            }
            i += 1;
        }
    }

    /**
     * @return list of blocks for this level.
     */
    public List<Block> getBlocks() {

        List<Block> blocksList = new ArrayList<Block>();

        // for the slides.
        Rectangle rec1 = new Rectangle(new Point(-30, -30), 0, 0);
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

        c = Color.gray;

        int hits = 2;
        for (int t = 40; t < 40 * 6; t += 40) {
            for (int i = 0; i < 500 - t; i += 50) {
                c = this.changeColor(t / 40);
                Block b1 = new Block(new Rectangle(new Point(
                        730 - i, 140 + t / 2), 50, 20),
                        hits, c);
                blocksList.add(b1);
            }
            hits = 1;
        }
        return blocksList;

    }

    /**
     * @param lineNumber - the line that we want to draw.
     * @return the color we will draw with.
     */
    private Color changeColor(int lineNumber) {
        Color c = Color.GRAY;
        if (lineNumber == 2) {
            c = Color.RED;
        }
        if (lineNumber == 3) {
            c = Color.YELLOW;
        }
        if (lineNumber == 4) {
            c = Color.BLUE;
        }
        if (lineNumber == 5) {
            c = Color.WHITE;
        }
        return c;
    }
}
