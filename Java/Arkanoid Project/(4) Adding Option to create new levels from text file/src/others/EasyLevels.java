import java.util.List;

/**
 * This program is responsible to run the hard levels,
 * after the user choose this option.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-18
 */
public class EasyLevels implements Task<Void> {

    private List<LevelInformation> levels;
    private GameFlow g;

    /**
     * @param levelsToSet - the hard levels to run.
     * @param ga          - helps us running the game levels of this hard level.
     */
    public EasyLevels(List<LevelInformation> levelsToSet, GameFlow ga) {
        this.levels = levelsToSet;
        this.g = ga;
        //this.animationRunner = anRunner;
    }

    @Override
    public Void run() {
        this.g.runLevels2(this.levels);
        return null;
    }

}
