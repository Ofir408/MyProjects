import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

/**
 * GameLevel1 Class - Direct hit.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-23
 */
public class GameLevel1 implements LevelInformation {
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
    public GameLevel1() {
        this.levelName = "Direct hit";
        this.ballsNumber = 1;
        this.paddleSpeed = 5;
        this.paddleWidth = 80;
        // need to set the background with the function setBackground
        this.numberOfBlocksToRemove = 1;
        // change 300 when the screen is 800 x 600.
        Rectangle r = new Rectangle(new Point(285 + 100, 150), 30, 30);
        this.blocksList.addAll(new FirstBackGround().getBlockList());
        this.blocksList.add(new Block(r, Color.RED, Color.BLACK));

        this.ballVelocitiesList = this.initialBallVelocities();
        this.background = new FirstBackGround();
    }

    @Override
    public int numberOfBalls() {
        return this.ballsNumber;
    }

    @Override
    public List<Velocity> initialBallVelocities() {
        // this function initial Ball Velocities and return a  the list
        // of velocities.
        List<Velocity> velList = new ArrayList<Velocity>();
        velList.add(new Velocity(0, -5));
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
        for (int i = 0; i < this.ballVelocitiesList.size(); i += 1) {
            Ball newBall = new Ball(400, 450, 5, Color.WHITE);
            newBall.setVelocity(this.ballVelocitiesList.get(i));
            ballsList.add(newBall);
        }
        return ballsList;
    }
}
