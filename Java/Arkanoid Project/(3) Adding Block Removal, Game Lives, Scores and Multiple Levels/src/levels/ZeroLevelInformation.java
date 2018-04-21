import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

/**
 * This program is a ZeroLevelInformation class.
 * this class used us to define the game level for the initial level.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-27
 */
public class ZeroLevelInformation implements LevelInformation {

    private int ballsNumber;
    private int paddleSpeed;
    private int paddleWidth;
    private String levelName;
    private Sprite background; // change it.
    private int numberOfBlocksToRemove;
    private List<Block> blocksList = new ArrayList<Block>();
    private List<Velocity> ballVelocitiesList = new ArrayList<Velocity>();

    /**
     * Default constructor to GameLevel1.
     */
    public ZeroLevelInformation() {
        this.levelName = "Zero level";
        this.background = new ZeroBackground();
        this.ballsNumber = 2;
        this.paddleSpeed = 5;
        this.paddleWidth = 80;
        this.blocksList = new ZeroBackground().getBlocksList();
        this.numberOfBlocksToRemove = blocksList.size();
        this.ballVelocitiesList = this.initialBallVelocities();

    }

    @Override
    public int numberOfBalls() {
        return this.ballsNumber;
    }

    @Override
    public List<Velocity> initialBallVelocities() {
        // 2 balls in zero level
        List<Velocity> velList = new ArrayList<Velocity>();
        for (int i = 0; i < ballsNumber; i += 1) {
            velList.add(new Velocity(2, 4));
        }
        return velList;
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
        Ball ball1 = new Ball(new Point(80, 100), 7,
                Color.orange);
        ball1.setVelocity(this.ballVelocitiesList.get(0));
        ballsList.add(ball1);

        Ball ball2 = new Ball(new Point(30, 220), 7,
                Color.GREEN);
        ball2.setVelocity(this.ballVelocitiesList.get(1));
        ballsList.add(ball2);

        return ballsList;
    }

    /**
     * @return ZeroLevelInformation.
     */
    public ZeroLevelInformation getZero() {
        return this;
    }

}
