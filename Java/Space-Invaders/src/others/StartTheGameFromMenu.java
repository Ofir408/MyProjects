
// after change StartTheGame

import biuoop.KeyboardSensor;

/**
 * This program is a StartTheGameFromMenu class. implements Task<Void>
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-09
 */
public class StartTheGameFromMenu extends GameFlow implements Task<Void> {
    /**
     * list of level to run.
     */
    //private List<LevelInformation> levelToRun;
    private Menu<Task<Void>> subMenu;
    private AnimationRunner runner;
    private KeyboardSensor keyboard;
    //private int numberOfLive;

    /**
     * @param newSubMenu   -  Menu<Task<Void>>.
     * @param anirunner - AnimationRunner helps in run.
     * @param ks            - KeyboardSensor, input of the user.
     */
    public StartTheGameFromMenu(AnimationRunner anirunner,
                                KeyboardSensor ks, Menu<Task<Void>> newSubMenu) {
        this.subMenu = newSubMenu;
        this.runner = anirunner;
        this.keyboard = ks;
    }

    @Override
    public Void run() {
        while (true) {
            this.runner.run(this.subMenu);
            Task<Void> result = this.subMenu.getStatus();
            result.run();
        }
    }
}
