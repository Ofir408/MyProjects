import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import biuoop.DrawSurface;

/**
 * FourBackGround Class - implements Sprite.
 * The second background in our game - "Final Four"
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-29
 */
public class FourBackGround implements Sprite {

    @Override
    public void drawOn(DrawSurface d) {
        d.setColor(Color.decode("#3399ff"));
        d.fillRectangle(0, 0, 800, 600);

        this.drawFirstCircles(d);
        //////////////////////////////
        d.setColor(Color.white);
        drawLines(d);
        this.drawSecondCircles(d);

    }

    @Override
    public void timePassed() {
    }

    @Override
    public void addToGame(GameLevel g) {
        g.addSprite(this);
    }

    /**
     * @return List of blocks for this level.
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

        // the other blocks
        int hits = 2;
        for (int t = 1; t < 8; t += 1) {

            for (int i = 0; i < 725; i += 40) {
                c = this.changeColor(t);
                Block b1 = new Block(new Rectangle(new Point(
                        740 - i, 140 + t * 20), 40, 20),
                        hits, c);
                blocksList.add(b1);
            }
            hits = 1;
        }
        return blocksList;
    }

    /**
     * @param lineNumber - the number of the line to draw.
     * @return the color of this line.
     */
    private Color changeColor(int lineNumber) {
        Color c = Color.CYAN;

        if (lineNumber == 1) {
            c = Color.DARK_GRAY;
        }
        if (lineNumber == 2) {
            c = Color.RED;
        }
        if (lineNumber == 3) {
            c = Color.yellow;
        }
        if (lineNumber == 4) {
            c = Color.GREEN;
        }
        if (lineNumber == 5) {
            c = Color.WHITE;
        }
        if (lineNumber == 6) {
            c = Color.PINK;
        }
        return c;
    }

    /**
     * draw the Circles.
     *
     * @param d - DrawSurface.
     */
    private void drawFirstCircles(DrawSurface d) {
        d.setColor(Color.white);
        int number = 10;
        int i = 0;
        while (i < number) {
            d.drawLine(100 + i * 8, 385, 50, 885 + 80 * i);
            i += 1;
        }
        d.fillCircle(100, 385, 20);
        d.fillCircle(120, 395, 25);
        d.setColor(Color.lightGray);
        d.fillCircle(130, 380, 28);
        d.setColor(Color.gray);
        d.fillCircle(155, 395, 30);
        d.fillCircle(135, 408, 20);
    }

    /**
     * @param d - DrawSurface.
     */
    private static void drawLines(DrawSurface d) {
        int number = 10;
        int i = 0;
        while (i < number) {
            d.drawLine(600 + i * 8, 485, 550, 985 + 80 * i);
            i += 1;
        }
    }

    /**
     * @param d - DrawSurface.
     */
    private void drawSecondCircles(DrawSurface d) {
        int xStart = 600;
        d.fillCircle(xStart, 485, 20);
        d.fillCircle(xStart + 15, 495, 25);
        d.setColor(Color.lightGray);
        d.fillCircle(xStart + 30, 480, 28);
        d.setColor(Color.gray);
        d.fillCircle(xStart + 55, 495, 30);
        d.fillCircle(xStart + 35, 508, 20);
    }

}
