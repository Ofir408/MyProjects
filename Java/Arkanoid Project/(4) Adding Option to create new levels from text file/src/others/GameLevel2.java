import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

/**
 * GameLevel2 Class - implements LevelInformation.
 * The second level in our game - "Wide Easy"
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-29
 */
public class GameLevel2 implements LevelInformation {

    private int ballsNumber;
    private int paddleSpeed;
    private int paddleWidth;
    private String levelName;
    private Sprite background; // change it.
    private int numberOfBlocksToRemove;
    private List<Block> blocksList = new ArrayList<Block>();
    private List<Velocity> ballVelocitiesList = new ArrayList<Velocity>();

    /**
     * Default constructor to GameLevel2.
     */
    public GameLevel2() {
        this.levelName = "Wide Easy";
        this.ballsNumber = 10;
        this.paddleSpeed = 1;
        this.paddleWidth = 720 - 60 - 40;
        // need to set the background with the function setBackground
        this.numberOfBlocksToRemove = 15;

        this.blocksList.addAll(new SecondBackGround().getBlocksList());

        this.ballVelocitiesList = this.initialBallVelocities();
        this.background = new SecondBackGround();
    }

    @Override
    public int numberOfBalls() {
        return this.ballsNumber;
    }

    @Override
    public List<Velocity> initialBallVelocities() {

        List<Velocity> listOfVelocities = new ArrayList<Velocity>();
        for (int i = 0; i < this.ballsNumber / 2; i++) {
            listOfVelocities.add(Velocity.fromAngleAndSpeed(10 + i * 12, 5));
            listOfVelocities.add(Velocity.fromAngleAndSpeed(-10 - i * 12, 5));
        }
        return listOfVelocities;
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
        return this.background;
    }

    @Override
    public List<Block> blocks() {
        return this.blocksList;
    }

    @Override
    public int numberOfBlocksToRemove() {
        return this.numberOfBlocksToRemove;
    }

    /**
     * @return list of balls initialized.
     */
    public List<Ball> getFinalListOfBalls() {
        List<Ball> ballsList = new ArrayList<Ball>();

        Color c = Color.WHITE;
        for (int i = 0; i < this.ballsNumber; i += 1) {
            Ball b = new Ball(397, 540, 6, c);
            b.setVelocity(this.ballVelocitiesList.get(i));

            ballsList.add(b);
        }
        return ballsList;
    }

}
