import biuoop.DrawSurface;
import biuoop.GUI;
import biuoop.Sleeper;

/**
 * This program is an AnimationRunner class.
 * The AnimationRunner takes an Animation object and runs it.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-24
 */
public class AnimationRunner {
    private GUI gui;
    private int framesPerSecond;

    /**
     * This is constructor function of AnimationRunner object.
     *
     * @param newGui             - GUI object for showing on the screen.
     * @param newFramesPerSecond - number of frames per each second.
     */
    public AnimationRunner(GUI newGui, int newFramesPerSecond) {
        this.gui = newGui;
        this.framesPerSecond = newFramesPerSecond;
    }

    /**
     * This is constructor function of AnimationRunner object.
     *
     * @param newGui - GUI object for showing on the screen.
     */
    public AnimationRunner(GUI newGui) {
        this.gui = newGui;
        this.framesPerSecond = 60;
    }

    /**
     * @return the gui.
     */
    public GUI getGui() {
        return this.gui;
    }

    /**
     * @return number of frames per each second.
     */
    public int getFramePerSecond() {
        return this.framesPerSecond;
    }

    /**
     * Takes an Animation object and runs it.
     *
     * @param animation - the animation for running.
     */
    public void run(Animation animation) {

        Sleeper sleeper = new Sleeper();
        int millisecondsPerFrame = 1000 / this.framesPerSecond;

        while (!animation.shouldStop()) {
            DrawSurface d = this.gui.getDrawSurface();
            long startTime = System.currentTimeMillis(); // timing

            animation.doOneFrame(d);

            // timing
            long usedTime = System.currentTimeMillis() - startTime;
            long milliSecondLeftToSleep =
                    millisecondsPerFrame - usedTime;
            if (milliSecondLeftToSleep > 0) {
                sleeper.sleepFor(milliSecondLeftToSleep);
            }
            this.gui.show(d);
        }

    }


}
