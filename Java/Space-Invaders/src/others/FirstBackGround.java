import java.awt.Color;
import java.awt.Image;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;

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


        //Rectangle rec1 = new Rectangle(new Point(0, 800), 800, 800);
        //Block b = new Block(rec1, 1, Color.BLACK, Color.BLACK);
        Image img = null;
        try {

            img = ImageIO.read(ClassLoader.getSystemClassLoader().getResourceAsStream("space.jpg")); // without
            // using
            // resources
            // library.
            d.drawImage(0, 0, img);
        } catch (Exception e) {
            Rectangle rec1 = new Rectangle(new Point(0, 800), 800, 800);
            // Block b = new Block(rec1, 1, Color.BLACK, Color.BLACK);
            d.setColor(Color.BLACK);
            d.fillRectangle((int) rec1.getUpperLeft().getX(),
                    (int) rec1.getUpperLeft().getY() - (int) rec1.getHeight(), (int) rec1.getWidth(),
                    (int) rec1.getHeight());
        }


        // draw the background of "Direct-hit" level.
        // Let's start draw the circles


    }

    @Override
    public void timePassed(double dt) {
    }

    @Override
    public void addToGame(GameLevel g) {
        g.addSprite(this);
    }

    /**
     * @return BlockSList.
     */
    public List<Block> getBlockList() {
        List<Block> blocksList = new ArrayList<>();

        Rectangle rec1 = new Rectangle(new Point(0, 0), 0, 0);  // need to check it
        Block b = new Block(rec1, 1, Color.BLACK, Color.BLACK);
        blocksList.add(b);

        Color c = Color.GRAY;
        Color stroke = Color.BLACK;
       /* Block[] slides = {new Block(new Rectangle(
                new Point(0, 40), 800, 20), 1, c, stroke),
                new Block(new Rectangle(new Point(
                        0, 600), 20, 600),
                        1, c, stroke),
                new Block(new Rectangle(new Point(
                        780, 600), 20, 600), 1, c, stroke),
                new Block(new Rectangle(new Point(
                        0, 620), 600, 20), 1, c, stroke)}; // the lowest block.*/

        Block[] slides = {new Block(new Rectangle(
                new Point(-30, -30), 0, 0), 1, c, stroke),
                new Block(new Rectangle(new Point(-30, -30), 0, 0),
                        1, c, stroke),
                new Block(new Rectangle(new Point(
                        -30, -30), 0, 0), 1, c, stroke),
                new Block(new Rectangle(new Point(
                        -30, -30), 0, 0), 1, c, stroke)};
        for (int i = 0; i < slides.length; i += 1) {
            blocksList.add(slides[i]);
        }

        return blocksList;
    }
}
