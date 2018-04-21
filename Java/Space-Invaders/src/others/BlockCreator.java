/**
 * BlockCreator interface BlockCreator is an interface of a factory-object that.
 * is used for creating blocks:
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-06-13
 */
public interface BlockCreator {

    /**
     * Create a block at the specified location.
     *
     * @param xpos - x start point position of the block.
     * @param ypos - y start point position of the block.
     * @return new Block with these positions.
     */
    Block create(int xpos, int ypos);

}
