import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

/**
 * This program is an Shield object.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-06-28
 */
public class Shield {

    private Point shieldUpperLeft;
    private List<List<Block>> blockListOfSheild = new ArrayList<>();

    /**
     * constructor of Shield.
     * @param upperLeft - upper left point.
     * @param blocks - list of blocks that components of the Shield.
     */
    public Shield(Point upperLeft, List<List<Block>> blocks) {
        this.blockListOfSheild = blocks;
        this.shieldUpperLeft = upperLeft;
    }

    /**
     * @param upperLeft - upper left of the start Shield. Initialize the blocks with
     *                  calling to helper function.
     */
    public Shield(Point upperLeft) {
        this.shieldUpperLeft = upperLeft;
        this.initializeSheild(this.shieldUpperLeft.getX(), this.shieldUpperLeft.getY());
    }

    /**
     * @param game      - GameLevel for adding the Shield to him.
     * @param bRemover  - BlockRemover remove hitted blocks.
     * @param sTracking - update score when Shield was hitted.
     */
    public void addShieldToGame(GameLevel game, BlockRemover bRemover, ScoreTrackingListener sTracking) {
        for (int i = 0; i < this.blockListOfSheild.size(); i++) {
            for (int h = 0; h < this.blockListOfSheild.get(0).size() - 1; h++) {

                Block current = this.blockListOfSheild.get(i).get(h);
                current.addToGame(game);
                current.addHitListener(bRemover);
                // current.addHitListener(sTracking);
            }
        }
    }

    /**
     * @param game      - GameLevel for adding the Shield to him.
     * @param bRemover  - BlockRemover remove hitted blocks.
     * @param sTracking - update score when Shield was hitted.
     */
    public void removeShieldToGame(GameLevel game, BlockRemover bRemover, ScoreTrackingListener sTracking) {
        for (int i = 0; i < this.blockListOfSheild.size(); i++) {
            for (int h = 0; h < this.blockListOfSheild.get(0).size() - 1; h++) {
                Block current = this.blockListOfSheild.get(i).get(h);
                if (current != null) {
                    current.removeFromGame(game);
                    current.removeHitListener(bRemover);
                }
                // current.addHitListener(sTracking);
            }
        }
    }

    /**
     * @param x - x upper left of new Shield.
     * @param y - x upper left of new Shield.
     */
    public void initializeSheild(double x, double y) {
        // this.shieldUpperLeft = new Point(x, y);
        double a = 4, b = 4;
        Block currentBlock = null;
        for (int i = 0; i < 3; i++) {
            this.blockListOfSheild.add(i, new ArrayList<Block>(15));
            for (int j = 0; j < 35; j++) {
                Rectangle currentRec = new Rectangle(
                        new Point(this.shieldUpperLeft.getX() + 4 * j, this.shieldUpperLeft.getY() + 4 * i), a, b);
                currentBlock = new Block(currentRec, 1, Color.green, Color.RED);
                currentBlock.setAsShieldBlock();

                // currentBlock.setLocation(i, j);
                this.blockListOfSheild.get(i).add(j, currentBlock);
            }
        }
    }

    /**
     *
     * @param b - block for checking.
     * @return true if the blockListOfSheild has this block.
     */
    public boolean hasInputBlock(Block b) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 35; j++) {
                Block cur = this.blockListOfSheild.get(i).get(j);
                if (cur.getCollisionRectangle() == b.getCollisionRectangle()) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     *
     * @return List<List<Block>> block list.
     */
    public List<List<Block>> getBlockList() {
        return this.blockListOfSheild;
    }

}
