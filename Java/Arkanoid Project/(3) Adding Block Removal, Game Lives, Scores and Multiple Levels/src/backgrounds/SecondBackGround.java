import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import biuoop.DrawSurface;

/**
 * SecondBackGround Class - implements Sprite.
 * The second background in our game - "Wide Easy"
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-29
 */
public class SecondBackGround implements Sprite {

    @Override
    public void drawOn(DrawSurface d) {

        d.setColor(Color.decode("#F0FFF0"));
        d.fillRectangle(0, 0, 800, 600);

        d.setColor(Color.yellow);
        d.fillCircle(100, 100, 50);
        d.setColor(Color.ORANGE);
        d.fillCircle(100, 100, 40);
        d.setColor(Color.yellow);
        d.fillCircle(100, 100, 30);
        for (int i = -40; i < 45; i++) {
            d.drawLine(100 + i, 100 + (int) Math.sqrt(2500 - i * i), -2 + (i + 40) * 8, 250); // need to change.
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
     * @return list of blocks in this level.
     */
    public List<Block> getBlocksList() {
        List<Block> blocksList = new ArrayList<Block>();


        // for the slides.
        Rectangle rec1 = new Rectangle(new Point(-30, -30), 0, 0);
        Block b = new Block(rec1, 1, Color.BLACK);
        blocksList.add(b);

        Color c = Color.GRAY;
        Block[] slides = {new Block(new Rectangle(
                new Point(0, 40), 800, 20), 1, c),
                new Block(new Rectangle(new Point(
                        0, 600), 23, 600),
                        1, c),
                new Block(new Rectangle(new Point(
                        772, 600), 27, 600), 1, c),
                new Block(new Rectangle(new Point(
                        0, 620), 600, 20), 1, c)}; // the lowest block.

        // ** check the following lines **
        for (int i = 0; i < slides.length; i += 1) {
            blocksList.add(slides[i]);
        }

        int blocksCounter = 0;


        // for the others block (except the slides that we have already drawn.
        for (int i = 75; i < 800; i += 50) { // need to change it when the screen is 800 x 600.

            c = this.colorChanger(blocksCounter);
            //if (800 - i == 775)
            Block b1 = new Block(new Rectangle(new Point(
                    800 - i - 3, 200), 50, 20),
                    1, c);

            blocksList.add(b1);
            blocksCounter += 1;
        }
        return blocksList;
    }

    /**
     * @param blocksCounter - how much blocks were drawed.
     * @return the Color for drawing the next block.
     */
    public Color colorChanger(int blocksCounter) {
        Color c = Color.decode("#33FFFF");
        if (blocksCounter == 2 || blocksCounter == 3) {
            c = Color.PINK;
        }
        if (blocksCounter == 4 || blocksCounter == 5) {
            c = Color.BLUE;
        }
        if (blocksCounter >= 6 && blocksCounter <= 8) {
            c = Color.GREEN;
        }
        if (blocksCounter == 9 || blocksCounter == 10) {
            c = Color.YELLOW;
        }
        if (blocksCounter == 11 || blocksCounter == 12) {
            c = Color.ORANGE;
        }
        if (blocksCounter == 13 || blocksCounter == 14) {
            c = Color.RED;
        }
        return c;
    }
}
