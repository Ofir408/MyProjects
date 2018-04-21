import java.util.ArrayList;
import java.util.List;

import biuoop.DrawSurface;

/**
 * This program is a ZeroBackground class.
 * that used us to zero level, initialize its background.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-27
 */
public class ZeroBackground implements Sprite {

    @Override
    public void drawOn(DrawSurface d) {
        for (int i = 0; i < this.getBlocksList().size(); i += 1) {
            Block b1 = getBlocksList().get(i);
            b1.drawOn(d, b1.getColor());
        }
    }

    @Override
    public void addToGame(GameLevel g) {
        g.addSprite(this);
    }

    /**
     * @return list of all this blocks in this background, except his sides.
     */
    public List<Block> getBlocksList() {
        List<Block> blocksList = new ArrayList<>();
        return blocksList;
    }

    @Override
    public void timePassed(double dt) {
        // TODO Auto-generated method stub

    }

}
