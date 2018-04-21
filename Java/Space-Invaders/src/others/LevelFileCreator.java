import java.awt.Color;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;

/**
 * LevelFileCreator class for bulid level from the file. Implement
 * LevelInformation because finally need to return list of levelInformation.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-06-17
 */
public class LevelFileCreator implements LevelInformation {

    private String levelName;
    private int ballsNumber, blockToRemoveNumber, paddleSpeed, paddleWidth;
    private List<Velocity> velList = new ArrayList<Velocity>();
    private List<Block> blocksList = new ArrayList<Block>();
    private Color backgroundColor;
    private Image backgroundImage;

    /**
     * @param name              - level name.
     * @param ballVel           - ball velocities list.
     * @param newPaddleWidth    - width of the paddle.
     * @param newPaddleSpeed    - speed of the paddle.
     * @param blockRemoveNumber - how much block we need to remove in order to pass this
     *                          level.
     * @param newBlocksList     - list of blocks in this level.
     */
    private LevelFileCreator(String name, List<Velocity> ballVel, int newPaddleWidth, int newPaddleSpeed,
                             int blockRemoveNumber, List<Block> newBlocksList) {
        this.levelName = name;
        this.velList = ballVel;
        this.velList = this.returnVelList();
        this.paddleWidth = newPaddleWidth;
        this.paddleSpeed = newPaddleSpeed / 60;
        this.blockToRemoveNumber = blockRemoveNumber;
        this.blocksList = this.addBorders();
        this.blocksList.addAll(newBlocksList);
        this.ballsNumber = this.velList.size();
    }

    /**
     * @param name              - level name.
     * @param ballVel           - ball velocities list.
     * @param newPaddleWidth    - width of the paddle.
     * @param newPaddleSpeed    - speed of the paddle.
     * @param blockRemoveNumber - how much block we need to remove in order to pass this
     *                          level.
     * @param newBlocksList     - list of blocks in this level.
     * @param colorBackground   - the background of this level (is color).
     */
    public LevelFileCreator(String name, List<Velocity> ballVel, int newPaddleWidth, int newPaddleSpeed,
                            int blockRemoveNumber, List<Block> newBlocksList, Color colorBackground) {
        this(name, ballVel, newPaddleWidth, newPaddleSpeed, blockRemoveNumber, newBlocksList);

        this.backgroundColor = colorBackground;
    }

    /**
     * @param name              - level name.
     * @param ballVel           - ball velocities list.
     * @param newPaddleWidth    - width of the paddle.
     * @param newPaddleSpeed    - speed of the paddle.
     * @param blockRemoveNumber - how much block we need to remove in order to pass this
     *                          level.
     * @param newBlocksList     - list of blocks in this level.
     * @param imageFilePath     - the background of this level is image.
     */
    public LevelFileCreator(String name, List<Velocity> ballVel, int newPaddleWidth, int newPaddleSpeed,
                            int blockRemoveNumber, List<Block> newBlocksList, File imageFilePath) {

        this(name, ballVel, newPaddleWidth, newPaddleSpeed, blockRemoveNumber, newBlocksList);

        // convert file to image
        BufferedImage img = null;
        try {
            img = ImageIO.read(imageFilePath);
            this.backgroundImage = img;
        } catch (IOException e) {
            System.out.println(
                    "IO exception in LevelFileCreator constructor," + " can't load the image from your file path ");
        }
    }

    /**
     * @param name              - level name.
     * @param ballVel           - ball velocities list.
     * @param newPaddleWidth    - width of the paddle.
     * @param newPaddleSpeed    - speed of the paddle.
     * @param blockRemoveNumber - how much block we need to remove in order to pass this
     *                          level.
     * @param newBlocksList     - list of blocks in this level.
     * @param img               - Image for the background.
     */
    public LevelFileCreator(String name, List<Velocity> ballVel, int newPaddleWidth, int newPaddleSpeed,
                            int blockRemoveNumber, List<Block> newBlocksList, Image img) {

        this(name, ballVel, newPaddleWidth, newPaddleSpeed, blockRemoveNumber, newBlocksList);
        this.backgroundImage = img;
    }

    @Override
    public int numberOfBalls() {
        return this.ballsNumber;
    }

    @Override
    public List<Velocity> initialBallVelocities() {
        return this.velList;
    }

    @Override
    public int paddleSpeed() {
        return this.paddleSpeed;
    }

    @Override
    public int paddleWidth() {
        return this.paddleWidth;
    }

    @Override
    public String levelName() {
        return this.levelName;
    }

    @Override
    public Sprite getBackground() {
        if (this.backgroundImage != null) {
            return new BackgroundFromFile(this.backgroundImage);
        }
        return new BackgroundFromFile(this.backgroundColor);
    }

    @Override
    public List<Block> blocks() {
        return this.blocksList;
    }

    @Override
    public int numberOfBlocksToRemove() {
        return this.blockToRemoveNumber;
    }

    /**
     * @return the list of velocities
     */
    private List<Velocity> returnVelList() {
        List<Velocity> toReturn = new ArrayList<>();
        for (Velocity v : this.velList) {
            toReturn.add(new Velocity(v.getDx() / 59.9, v.getDy() / 59.9));
        }
        return toReturn;
    }

    /**
     * @return list of block (the boarders).
     */
    private List<Block> addBorders() {
        List<Block> bl = new ArrayList<>();
        Color stroke = Color.gray;
        // for the slides.
        Rectangle rec1 = new Rectangle(new Point(-30, -30), 0, 0);
        Block b = new Block(rec1, 1, Color.BLACK, stroke);
        bl.add(b);

        Color c = Color.GRAY;
        /// public Block(final Rectangle r, final Color newColor, Color stroke) {
        Block[] slides = {new Block(new Rectangle(new Point(0, 40), 800, 20), 1000, c, stroke),
                new Block(new Rectangle(new Point(0, 600), 20, 600), 1000, c, stroke),
                new Block(new Rectangle(new Point(780, 600), 20, 600), 1000, c, stroke),
                new Block(new Rectangle(new Point(0, 620), 600, 20), 1000, c, stroke)};

        for (int i = 0; i < slides.length; i += 1) {
            bl.add(slides[i]);
        }
        return bl;
    }
}
