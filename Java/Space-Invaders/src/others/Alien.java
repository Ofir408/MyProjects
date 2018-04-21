import java.awt.Color;
import java.awt.Image;
import java.io.IOException;

import javax.imageio.ImageIO;

/**
 * This program is an Alien object.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-06-26
 */
public class Alien extends Block implements Shooter {
    /**
     * speed of this Alien (moves left or right).
     */
    private double speed;
    private Rectangle originalRec;

    /**
     * @param r             - Rectangle of the block.
     * @param maxHitsNumber - hit points until should be deleted.
     * @param stroke        - stroke color of the block.
     */
    public Alien(Rectangle r, int maxHitsNumber, Color stroke) {
        super(r, 1, Alien.getImageFromFile(), null);
        originalRec = new Rectangle(r.getUpperLeft(), r.getWidth(), r.getHeight());
    }


    /**
     * @return the Image of this file.
     */
    private static Image getImageFromFile() {
        Image img = null;
        try {

            img = ImageIO.read(ClassLoader.getSystemClassLoader().getResourceAsStream("alien.png")); // without
            // using
            // resources
            // library.
        } catch (IOException e) {
            System.out.println("There is a problem to find this image");
        }
        return img;
    }

    // Shooting Every 0.5 seconds
    @Override
    public void shot(GameLevel gameLevel) {
        double x = this.getCollisionRectangle().getUpperLeft().getX() + this.getCollisionRectangle().getWidth() / 2;
        double y = this.getCollisionRectangle().getUpperLeft().getY() - this.getCollisionRectangle().getHeight();
        Point startPoint = new Point(x, y);
        Ball ballToAdd = new Ball(startPoint, 3, Color.RED);
        ballToAdd.setVelocity(0, 3);
        ballToAdd.setGameEnvironment(gameLevel.getGameEnvironment());
        ballToAdd.addToGame(gameLevel);
        ballToAdd.setToAlienBall();
        gameLevel.addBall(ballToAdd);
    }

    /**
     * add option to set the location of the alien , (of the block).
     *
     * @param newXpoint - x new point of the rectangle after change Alien location.
     * @param newYpoint - y new point of the rectangle after change Alien location.
     */
    public void setAlienLocation(double newXpoint, double newYpoint) {
        super.setRectangle(new Point(newXpoint, newYpoint));
    }

    /**
     * @param newSpeed - new speed to set to the Alien
     */
    public void setSpeed(double newSpeed) {
        this.speed = newSpeed;
    }

    /**
     * Adding the Alien to the game when we refer to him as a block.
     *
     * @param gameLevel - our game.
     */
    public void addAlienToGame(GameLevel gameLevel) {
        super.addToGame(gameLevel);
    }

    /**
     * set the rec of this Alien to his origin.
     */
    public void setRecOfAlienToOriginal() {
        this.setBlockRectangle(this.originalRec);
    }

    /**
     * The Alien moves right/left according his speed. When positive speed means
     * moving to right. Otherwise -> moves left.
     */
    public void moveAlien() {

        if (this.speed > 0) {
            // Moves right.
            if (800 - super.getCollisionRectangle().getUpperLeft().getX()
                    - super.getCollisionRectangle().getWidth() > this.speed) {
                // move him right according his speed.
                this.setAlienLocation(super.getCollisionRectangle().getUpperLeft().getX() + this.speed,
                        super.getCollisionRectangle().getUpperLeft().getY());
            } else {
                this.speed = this.speed * -1;
            }
        } else {
            if (super.getCollisionRectangle().getUpperLeft().getX() > Math.abs(this.speed)) {
                this.setAlienLocation(super.getCollisionRectangle().getUpperLeft().getX() + this.speed,
                        super.getCollisionRectangle().getUpperLeft().getY());
            } else {
                this.speed = this.speed * -1;
            }
        }
    }
}
