import biuoop.DrawSurface;
import biuoop.KeyboardSensor;

/**
 * KeyPressStoppableAnimation class that will wrap an existing animation and add
 * a "waiting-for-key" behavior to it.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-09
 */
public class KeyPressStoppableAnimation implements Animation {

    private KeyboardSensor keyboard;
    private String keyForStopAnimation;
    private Animation animation;
    private boolean isAlreadyPressed;

    /**
     * @param sensor       - KeyboardSensor to check if the key is pressed.
     * @param key          - a String key that if it pressed it means we need to stop
     *                     this animation.
     * @param newAnimation - the Animation to show.
     */
    public KeyPressStoppableAnimation(KeyboardSensor sensor, String key, Animation newAnimation) {
        this.keyboard = sensor;
        this.keyForStopAnimation = key;
        this.animation = newAnimation;
        isAlreadyPressed = true;
    }

    @Override
    public void doOneFrame(DrawSurface d, double dt) {
        this.animation.doOneFrame(d, dt);
    }

    @Override
    public boolean shouldStop() {
        boolean finishKeyPressed = this.animation.shouldStop();
        if (finishKeyPressed) {
            if (!this.isAlreadyPressed) {
                return true;
            }
        } else {
            this.isAlreadyPressed = false;
        }
        return false;
    }

}
