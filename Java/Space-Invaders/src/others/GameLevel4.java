import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

/**
 * GameLevel3 Class - implements LevelInformation.
 * The second level in our game - "Green 3"
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-29
 */
public class GameLevel4 implements LevelInformation {

    private int ballsNumber;
    private int paddleSpeed;
    private int paddleWidth;
    private String levelName;
    private Sprite background;
    private int numberOfBlocksToRemove;
    private List<Block> blocksList = new ArrayList<Block>();
    private List<Velocity> ballVelocitiesList = new ArrayList<Velocity>();

    /**
     * Default constructor to GameLevel4.
     */
    public GameLevel4() {
        this.levelName = "Final Four";
        this.ballsNumber = 3;
        this.paddleSpeed = 5;
        this.paddleWidth = 80;
        // need to set the background with the function setBackground
        this.numberOfBlocksToRemove = 105;

        this.blocksList.addAll(new FourBackGround().getBlocks());

        this.ballVelocitiesList = this.initialBallVelocities();
        this.background = new FourBackGround();
    }

    @Override
    public int numberOfBalls() {
        return this.ballsNumber;
    }

    @Override
    public List<Velocity> initialBallVelocities() {
        List<Velocity> velocities = new ArrayList<Velocity>();
        //velocities.add(new Velocity(4.123456, -3.1231));
       // velocities.add(new Velocity(0, -Math.sqrt(29)));
       //
       // velocities.add(new Velocity(-4.123456, -3.1231));
        velocities.add(Velocity.fromAngleAndSpeed(47.5, 5.139));
        velocities.add(Velocity.fromAngleAndSpeed(0, 5.123456));
        velocities.add(Velocity.fromAngleAndSpeed(-47.5, 5.138));
        return velocities;
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

        for (int i = 0; i < this.ballsNumber; i += 1) {
            Color c = Color.WHITE;
            Ball b = new Ball(397, 540, 6, c);
            b.setVelocity(this.ballVelocitiesList.get(i));
            ballsList.add(b);
        }
        return ballsList;

    }

}
