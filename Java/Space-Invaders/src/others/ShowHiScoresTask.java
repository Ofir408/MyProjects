
/**
 * This program is a ShowHiScoresTask class.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-09
 */
public class ShowHiScoresTask implements Task<Void> {

    private AnimationRunner runner;
    private Animation highScoresAnimation;

    /**
     * @param newRunner              - AnimationRunner helps us run the animation.
     * @param newHighScoresAnimation - animation for running.
     */
    public ShowHiScoresTask(AnimationRunner newRunner, Animation newHighScoresAnimation) {
        this.runner = newRunner;
        this.highScoresAnimation = newHighScoresAnimation;
    }

    @Override
    public Void run() {
        this.runner.run(this.highScoresAnimation);
        return null;
    }

}
