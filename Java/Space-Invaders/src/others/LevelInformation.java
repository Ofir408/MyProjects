import java.util.List;

/**
 * This program is an interface LevelInformation.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-26
 */
public interface LevelInformation {
    /**
     * @return integer, number of balls in this level.
     */
    int numberOfBalls();

    /**
     * The initial velocity of each ball.
     * Note that initialBallVelocities().size() == numberOfBalls()
     *
     * @return list of velocities after initialized.
     */
    List<Velocity> initialBallVelocities();

    /**
     * @return the speed of the paddle.
     */
    int paddleSpeed();

    /**
     * @return the width of the paddle.
     */
    int paddleWidth();

    /**
     * the level name will be displayed at the top of the screen.
     *
     * @return the level name.
     */
    String levelName();

    /**
     * @return a sprite with the background of the level
     */
    Sprite getBackground();

    /**
     * The Blocks that make up this level, each block contains
     * its size, color and location.
     *
     * @return list of blocks that contain in this level,
     * each block has
     * details of its size, color and location.
     */
    List<Block> blocks();

    /**
     * Number of levels that should be removed
     * before the level is considered to be "cleared".
     * This number should be <= blocks.size();
     *
     * @return the number of levels that should be removed.
     */
    int numberOfBlocksToRemove();

}
