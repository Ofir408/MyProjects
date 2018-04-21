import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import biuoop.DrawSurface;

/**
 * This program is a ZeroBackground class.
 * that used us to zero level, initialize its background.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-27
 */
public class ZeroBackground implements Sprite {

    @Override
    public void drawOn(DrawSurface d) {
        for (int i = 0; i < this.getBlocksList().size(); i += 1) {
            Block b1 = getBlocksList().get(i);
            b1.drawOn(d, b1.getColor());
        }
    }

    @Override
    public void timePassed() {
    }

    @Override
    public void addToGame(GameLevel g) {
        g.addSprite(this);
    }

    /**
     * @return list of all this blocks in this background, except his sides.
     */
    public List<Block> getBlocksList() {
        List<Block> blocksList = new ArrayList<>();

        // blue background.
        Rectangle rec1 = new Rectangle(new Point(0, 600), 600, 600);  // need to check it
        Block b = new Block(rec1, 1, Color.BLUE);
        blocksList.add(b);

        Color c = Color.GRAY;
        Block[] slides = {new Block(new Rectangle(
                new Point(0, 40), 600, 20), 1, c),
                new Block(new Rectangle(new Point(
                        0, 600), 20, 600),
                        1, c),
                new Block(new Rectangle(new Point(
                        580, 600), 20, 600), 1, c),
                new Block(new Rectangle(new Point(
                        0, 600), 600, 20), 1, c)}; // the lowest block.

        // ** check the following lines **
        for (int i = 0; i < slides.length; i += 1) {
            blocksList.add(slides[i]);
        }


        c = Color.gray;
        for (int i = 0; i < 440; i += 40) {
            Block b1 = new Block(new Rectangle(new Point(
                    500 - i, 140), 40, 20),
                    1, c);
            blocksList.add(b1);
        }

        // another row - first row - orange color
        c = Color.ORANGE;
        for (int i = 0; i < 400; i += 40) {
            Block b2 = new Block(new Rectangle(new Point(
                    500 - i, 160), 40, 20),
                    1, c);
            blocksList.add(b2);
        }
        // second row -  red color
        c = Color.RED;
        for (int i = 0; i < 360; i += 40) {
            Block b3 = new Block(new Rectangle(new Point(
                    500 - i, 180), 40, 20),
                    1, c);
            blocksList.add(b3);
        }
        c = Color.darkGray;
        for (int i = 0; i < 320; i += 40) {
            Block b4 = new Block(new Rectangle(new Point(
                    500 - i, 200), 40, 20),
                    1, c);
            blocksList.add(b4);
        }
        c = Color.CYAN;
        for (int i = 0; i < 280; i += 40) {
            Block b5 = new Block(new Rectangle(new Point(
                    500 - i, 220), 40, 20),
                    1, c);
            blocksList.add(b5);
        }
        c = Color.GREEN;
        for (int i = 0; i < 240; i += 40) {
            Block b6 = new Block(new Rectangle(new Point(
                    500 - i, 240), 40, 20),
                    1, c);
            blocksList.add(b6);

        }
        c = Color.PINK;
        for (int i = 0; i < 200; i += 40) {
            Block b7 = new Block(new Rectangle(new Point(
                    500 - i, 260), 40, 20),
                    1, c);
            blocksList.add(b7);
        }
        return blocksList;
    }

}
