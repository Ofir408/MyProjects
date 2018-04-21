import java.awt.Color;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import biuoop.DrawSurface;

/**
 * BackgroundFromFile Class - implements Sprite. Used in LevelFileCreator class.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-29
 */
public class BackgroundFromFile implements Sprite {
    private Image image = null;
    private Color color = null;
    private int screenWidth = 800;
    private int screenLength = 600;

    /**
     * @param img - image for the background of this level.
     */
    public BackgroundFromFile(Image img) {
        this.image = img;
    }

    /**
     * @param imageFilePath - file path with image inside.
     */
    public BackgroundFromFile(File imageFilePath) {
        // convert file to image
        BufferedImage img = null;

        try {
            img = ImageIO.read(imageFilePath);
            this.image = img;
        } catch (IOException e) {
            System.out.println(
                    "IO exception in BackgroundFromFile constructor,"
                            + " can't load the image from your file path ");
        }
    }

    /**
     * @param newColor - Color for the background of this level.
     */
    public BackgroundFromFile(Color newColor) {
        this.color = newColor;
    }

    @Override
    public void drawOn(DrawSurface d) {
        if (this.color != null) {
            d.setColor(this.color);
            d.fillRectangle(0, 0, screenWidth, screenLength);
        }
        d.drawImage(0, 0, this.image);
    }

    @Override
    public void timePassed(double dt) {
    }

    @Override
    public void addToGame(GameLevel g) {
        g.addSprite(this);
    }

    /**
     * @return this image background (maybe = null).
     */
    public Image getImage() {
        return this.image;
    }

    /**
     * @return this color background (maybe = null).
     */
    public Color getColor() {
        return this.color;
    }

}
