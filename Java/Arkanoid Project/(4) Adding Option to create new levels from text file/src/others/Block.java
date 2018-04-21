
//package threeass;

import java.awt.Color;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.imageio.ImageIO;

import biuoop.DrawSurface;


/**
 * That's Block class. that represents block that thing can collide with him.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-04-27
 */

public class Block implements Collidable, Sprite, HitNotifier {

    /**
     * the block is a rectangle object.
     */
    private Rectangle rec;
    /**
     * Each block should be initialized with a positive number of hit-points
     * number of hits in this block - initialized with the maximum number of
     * hits in the block, and then decrease in each more hit. when the number of
     * points is 0, it will stay 0)
     */
    private int hitPoints;

    /**
     * List of HitListeners, an interface that has date about hits that
     * occurred.
     */
    private List<HitListener> hitListenersList = new ArrayList<HitListener>();

    /**
     * the color of this block.
     */
    private Color color;

    /**
     * image to draw on the block.
     */
    private java.awt.Image image;

    /**
     * stroke of the block.
     */
    private Color blockStrokeColor;
    private Map<Integer, Image> mapImages = new HashMap<>();
    private Map<Integer, Color> mapColors = new HashMap<>();

    // functions

    /**
     * Constructor block object.
     *
     * @param r             - Rectangle object.
     * @param maxHitsNumber - initialized with the maximum number of hits in the block.
     * @param newColor      - the color of this block.
     * @param stroke        - the stroke of this block.
     */
    public Block(final Rectangle r, final int maxHitsNumber, final Color newColor, Color stroke) {
        this.rec = r;
        this.hitPoints = maxHitsNumber;
        this.color = newColor;
        this.blockStrokeColor = stroke;
    }

    /**
     * @param r                 - the rectangle presents this block.
     * @param maxHitsNumber     - hit points of this block.
     * @param fileImageForBlock - the file to get the image from.
     * @param stroke            - the stroke of this block.
     */
    public Block(Rectangle r, int maxHitsNumber, File fileImageForBlock, Color stroke) {
        this.rec = r;
        this.hitPoints = maxHitsNumber;
        this.blockStrokeColor = stroke;


        try {
            // try to get the image from the file.
            BufferedImage img = null;
            Path currentRelativePath = Paths.get("");
            String s = currentRelativePath.toAbsolutePath().toString().concat("/resources/")
                    .concat(fileImageForBlock.getPath().trim());
            img = ImageIO.read(new File(s));

            this.image = img;
        } catch (IOException e) {
            System.out.println("IO exception in block constructor,"
                    + " can't load the image from your file path(***) ");
        }
    }

    /**
     * @param r             - the rectangle presents this block.
     * @param maxHitsNumber - hit points of this block.
     * @param filesMap      - the files to get the images from.
     * @param stroke        - the stroke of this block.
     */
    public Block(Rectangle r, int maxHitsNumber, Map<Integer, File> filesMap, Color stroke) {
        this.rec = r;
        this.hitPoints = maxHitsNumber;
        this.blockStrokeColor = stroke;

        for (int i = 0; i < filesMap.size(); i += 1) {
            try {
                File currentFile = filesMap.get(i + 1);
                Path currentRelativePath = Paths.get("");
                String s = currentRelativePath.toAbsolutePath().toString().concat("/resources/")
                        .concat(currentFile.getPath().trim());
                Image img2 = ImageIO.read(new File(s));
                this.mapImages.put(i + 1, img2);

            } catch (IOException e) {
                System.out.println("IO exception in block constructor,"
                        + " can't load the image " + (i + 1)
                        + " from your file path(1) ");
            } catch (NullPointerException n) {
                System.out.println("IO exception in block constructor,"
                        + " can't load the image " + (i + 1)
                        + " from your file path(11) ");
            }
        }
    }

    /**
     * @param r             - the rectangle presents this block.
     * @param maxHitsNumber - hit points of this block.
     * @param colorsMap     - the map to get the images from.
     * @param stroke        - the stroke of this block.
     */
    public Block(Rectangle r, int maxHitsNumber, Color stroke, Map<Integer, Color> colorsMap) {
        this.rec = r;
        this.hitPoints = maxHitsNumber;
        this.blockStrokeColor = stroke;

        for (int i = 1; i < colorsMap.size() + 1; i += 1) {
            try {
                Color currentColor = colorsMap.get(i);
                this.mapColors.put(i, currentColor);
            } catch (Exception e) {
                System.out
                        .println("Exception in block constructor,"
                                + " can't load the color " + i + " from the input ");
            }
        }
    }

    /**
     * @param r             - Rectangle of block.
     * @param maxHitsNumber - hit points number.
     * @param stroke        - color of the stroke.
     * @param colorsMap     - Map<Integer, Color>
     * @param filesMap      -  Map<Integer, File>
     */
    public Block(Rectangle r, int maxHitsNumber, Color stroke, Map<Integer, Color> colorsMap,
                 Map<Integer, File> filesMap) {
        // new constructor

        this.rec = r;
        this.hitPoints = maxHitsNumber;
        this.blockStrokeColor = stroke;
        for (int i = 1; i < colorsMap.size() + filesMap.size() + 1; i++) {
            try {
                BufferedImage img = null;
                File currentFile = filesMap.get(i);

                Path currentRelativePath = Paths.get("");
                String s = currentRelativePath.toAbsolutePath().toString().concat("/resources/")
                        .concat(currentFile.getPath().trim());
                img = ImageIO.read(new File(s));

                this.mapImages.put(i, img);
            } catch (IllegalArgumentException ex) {
                // maybe it's color, continue check
                try {
                    Color currentColor = colorsMap.get(i);
                    this.mapColors.put(i, currentColor);
                } catch (Exception io) {
                    System.out.println("IllegalArgumentException exception in block constructor,"
                            + " can't load the image " + i
                            + " from your file path(***)");
                }
            } catch (IOException e) {
                // maybe it's color, continue check
                try {
                    Color currentColor = colorsMap.get(i);
                    this.mapColors.put(i, currentColor);
                } catch (Exception io) {
                    System.out.println("IO exception in block constructor,"
                            + " can't load the image " + i
                            + " from your file path(**");
                }
            } catch (NullPointerException n) {
                try {
                    Color currentColor = colorsMap.get(i);
                    this.mapColors.put(i, currentColor);
                } catch (Exception io) {
                    System.out.println("IO exception in block constructor,"
                            + " can't load the image " + i
                            + " from your file path (*)");
                }
            }
        }
    }

    /**
     * another Constructor block object.
     *
     * @param r        - Rectangle object.
     * @param stroke   - the stroke of this block.
     * @param newColor - the color of this block.
     */
    public Block(final Rectangle r, final Color newColor, Color stroke) {
        this.rec = r;
        this.color = newColor;
        this.blockStrokeColor = stroke;
    }

    /**
     * @return the "collision shape" of the object.
     */
    @Override
    public Rectangle getCollisionRectangle() {
        return this.rec;
    }

    /**
     * @return the number of the hit points (number of max hits - number of hits
     * until now) while decrease hitPoints in each more hit. But when
     * the number of points is 0, it will stay 0
     */
    public int getHitPointsNumber() {
        return this.hitPoints;
    }

    /**
     * @return the color of the current block
     */
    public Color getColor() {
        return this.color;
    }

    /**
     * this function calls when there is an hit. decrease hitPoints because
     * there was more hit. But when the number of points is 0, it will stay 0,
     * so if it's possible - decrease. Otherwise - do nothing.
     */
    public void decrease() {
        if (this.hitPoints > 0) {
            // it's possible to decrease
            this.hitPoints -= 1;
        } else {
            // do nothing, just confirm it's stay 0.
            this.hitPoints = 0;
        }
    }

    @Override
    /**
     * This function is called when there is an hit.
     *
     * @param collisionPoint
     *            - the collision Point between the block and the other object
     *            (ball).
     * @param currentVelocity
     *            - the velocity before the hit between them.
     * @param hitterBall
     *            - the hitter ball.
     * @return the new Velocity after the hit.
     */
    public Velocity hit(final Ball hitterBall, final Point collisionPoint, final Velocity currentVelocity) {
        // there was an hit so calls to decrease function
        this.decrease();
        this.notifyHit(hitterBall);

        double xVelocity = currentVelocity.getDx();
        double yVelocity = currentVelocity.getDy();

        if (this.hitX(collisionPoint)) {
            if (this.hitY(collisionPoint)) {
                // change both
                return new Velocity(-1 * xVelocity, -1 * yVelocity);
            } else {
                // hit x but y no, therefore change just x
                return new Velocity(-1 * xVelocity, yVelocity);
            }
        }
        // x no according first checking so change just y
        return new Velocity(xVelocity, -1 * yVelocity);
    } // */
    // }

    /**
     * this function checks if there is intersection with X.
     *
     * @param collisionPoint - the collision Point between the block and the ball.
     * @return true if the collision Point hit X
     */
    public final boolean hitX(final Point collisionPoint) {
        if (this.rec.getUpperLeft().getX() == collisionPoint.getX()
                || this.rec.getUpperLeft().getX() + this.rec.getWidth() == collisionPoint.getX()) {
            if (collisionPoint.getY() <= this.rec.getUpperLeft().getY()
                    && collisionPoint.getY() >= this.rec.getUpperLeft().getY() - this.rec.getHeight()) {
                return true;
            }
        }
        return false;
    }

    /**
     * this function checks if there is intersection with Y.
     *
     * @param collisionPoint - the collision Point between the block and the ball.
     * @return true if the collision Point hit Y.
     */
    public final boolean hitY(final Point collisionPoint) {
        if (this.rec.getUpperLeft().getY() == collisionPoint.getY()
                || this.rec.getUpperLeft().getY() - this.rec.getHeight() == collisionPoint.getY()) {
            if (collisionPoint.getX() >= this.rec.getUpperLeft().getX()
                    && collisionPoint.getX() <= this.rec.getUpperLeft().getX() + this.rec.getWidth()) {
                return true;
            }
        }
        return false;
    }

    /**
     * The function draw the current block on the given DrawSurface. void
     * function.
     *
     * @param surface - input DrawSurface that helps us to draw the block object
     *                about him.
     */
    // draw the ball on the given DrawSurface
    @Override
    public void drawOn(final DrawSurface surface) {
        // draw the rectangle of this block

        if (this.image != null) {
            // there is just one image to draw
            surface.drawImage((int) this.rec.getUpperLeft().getX(),
                    (int) this.rec.getUpperLeft().getY() - (int) this.rec.getHeight(), this.image);
        } else if (!this.mapImages.isEmpty() || !this.mapColors.isEmpty()) {
            if (this.hitPoints > 0) {
                if (this.mapImages.get(this.hitPoints) != null) {
                    // there is an image to draw on the block
                    surface.drawImage((int) this.rec.getUpperLeft().getX(),
                            (int) this.rec.getUpperLeft().getY() - (int) this.rec.getHeight(),
                            this.mapImages.get(this.hitPoints));
                } else {
                    // no image, checks if there is special color
                    if (this.mapColors.get(this.hitPoints) != null) {
                        surface.setColor(this.mapColors.get(this.hitPoints));
                        surface.fillRectangle((int) this.rec.getUpperLeft().getX(),
                                (int) this.rec.getUpperLeft().getY() - (int) this.rec.getHeight(),
                                (int) this.rec.getWidth(), (int) this.rec.getHeight());
                    }
                }
            }
        } else {
            // no one image, no one color/image in his map, therefore:
            // System.out.println("color is : " + this.color);
            surface.setColor(this.color);
            surface.fillRectangle((int) this.rec.getUpperLeft().getX(),
                    (int) this.rec.getUpperLeft().getY() - (int) this.rec.getHeight(), (int) this.rec.getWidth(),
                    (int) this.rec.getHeight());
        }

        this.drawBlockStroke(surface);

        // this.drawNumberHits(surface, Color.WHITE);
    }

    /**
     * The function draw the current block on the given DrawSurface. void
     * function.
     *
     * @param surface  - input DrawSurface that helps us to draw the block object
     *                 about him.
     * @param newColor - new color of the circle to draw.
     */
    // draw the ball on the given DrawSurface
    public void drawOn(final DrawSurface surface, final Color newColor) {
        surface.setColor(newColor);
        surface.fillRectangle((int) this.rec.getUpperLeft().getX(),
                (int) this.rec.getUpperLeft().getY() - (int) this.rec.getHeight(), (int) this.rec.getWidth(),
                (int) this.rec.getHeight());
    }

    /**
     * The function draw the current block on the given DrawSurface. void
     * function.
     *
     * @param surface
     *            - input DrawSurface that helps us to draw the block object
     *            about him.
     * @param newColor
     *            - new color of the circle to draw.
     *
     *            // draw the ball on the given DrawSurface public void
     *            drawBackground(final DrawSurface surface, final Color
     *            newColor) { System.out.println("background");
     *            surface.setColor(newColor); surface.fillRectangle((int)
     *            this.rec.getUpperLeft().getX(), (int)
     *            this.rec.getUpperLeft().getY() - (int) this.rec.getHeight(),
     *            (int) this.rec.getWidth(), (int) this.rec.getHeight()); //
     *            this.drawNumberHits(surface, Color.WHITE); }
     */

    /**
     * The hit point will be indicated visually on the block (as a white number
     * / letter in the middle of the block). block with 0 hit-points should have
     * a 'X' displayed on it
     *
     * @param surface  - input DrawSurface that helps us to draw the block object
     *                 about him.
     * @param newColor - new color of the circle to draw.
     */
    public void drawNumberHits(final DrawSurface surface, final Color newColor) {
        surface.setColor(Color.WHITE);
        // check where is the middle of the block,
        // because we want to draw there.
        double xPlace = this.rec.getUpperLeft().getX() + this.rec.getWidth() / 2.00;
        double yPlace = this.rec.getUpperLeft().getY() - this.rec.getHeight() / 2.00;
        if (this.hitPoints < 1) {
            // draw "X"
            surface.drawText((int) xPlace, (int) yPlace, "X", 15);
        } else {
            // more than 1 - so the number is displayed on the middle
            // of the current block.
            surface.drawText((int) xPlace, (int) yPlace, String.valueOf(this.hitPoints), 15);
        }
    }

    /**
     * @param surface - the DrawSurface.
     */
    public void drawBlockStroke(DrawSurface surface) {
        try {
            // Draw the stroke of this block
            surface.setColor(this.blockStrokeColor);
            Point start = new Point(this.rec.getUpperLeft().getX() + this.rec.getWidth(),
                    this.rec.getUpperLeft().getY());
            Point end = new Point(start.getX(), start.getY() - this.rec.getHeight());
            surface.drawLine((int) start.getX(), (int) start.getY(), (int) end.getX(),
                    (int) end.getY());
            // draw stroke for the left side:
            surface.drawLine((int) start.getX() - (int) this.rec.getWidth(), (int) start.getY(),
                    (int) start.getX() - (int) this.rec.getWidth(), (int) end.getY());

            // draw Horizontal line from above

            Point upperL = this.rec.getUpperLeft();
            surface.drawLine((int)
                            upperL.getX(), (int) upperL.getY(), (int) (upperL.getX() + this.rec.getWidth()),
                    (int) upperL.getY());

            // draw Horizontal line from down
            surface.drawLine((int) upperL.getX(), (int) (upperL.getY() - this.rec.getHeight()),
                    (int) (upperL.getX() + this.rec.getWidth()), (int) (upperL.getY()
                            - this.rec.getHeight()));

        } catch (NullPointerException n) {
            System.out.print("");
        }

    }

    @Override
    public void timePassed(double dt) {
    }

    @Override
    public final void addToGame(final GameLevel g) {
        // Block is both a sprite and a collidable
        // when we add a Block we need to call both
        // addSprite and addCollidable.
        g.addCollidable(this);
        g.addSprite(this);
    }

    /**
     * @param game - the Game object that we will remove the block from his.
     */
    public void removeFromGame(final GameLevel game) {
        // Block is both a sprite and a collidable
        // when we remove a Block we need to call both
        // removeSprite and removeCollidable.
        game.removeCollidable(this);
        game.removeSprite(this);
    }

    @Override
    public void addHitListener(HitListener hl) {
        this.hitListenersList.add(hl);
    }

    @Override
    public void removeHitListener(HitListener hl) {
        this.hitListenersList.remove(hl);
    }

    /**
     * Notify all listeners about a hit event.
     *
     * @param hitter - the ball that his the current block.
     */
    private void notifyHit(Ball hitter) {
        // Make a copy of the hitListeners before iterating over them.
        List<HitListener> listeners = new ArrayList<HitListener>(this.hitListenersList);
        // Notify all listeners about a hit event:
        for (HitListener hl : listeners) {
            hl.hitEvent(this, hitter);
        }
    }

    /**
     * none.
     */
    public void printBlock() {
        System.out.println("**** Block printing ****");
        System.out.println("hit point number is: " + this.hitPoints);
        System.out.println("Block color is : " + this.color);
        System.out.println("stroke color: " + this.blockStrokeColor);
        System.out.println("One image is: " + this.image);
        System.out.println("color Map is: " + this.mapColors);
        System.out.println("Images Map is: " + this.mapImages);
        System.out.println("**** finish print **** ");
    }

}
