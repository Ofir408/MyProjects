import java.util.Map;

/**
 * BlocksFromSymbolsFactory Class You will thus need a mechanism (object) with a.
 * method that will get a symbol and create the desired block
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-06-13
 */
public class BlocksFromSymbolsFactory {

    private Map<String, Integer> spacerWidths;
    private Map<String, BlockCreator> blockCreators;

    /**
     * Constructor method to BlocksFromSymbolsFactory.
     *
     * @param spacerWidthsMap  - map of string to integers, that represents the widths of
     *                         spaces.
     * @param blockCreatorsMap - map of string to BlockCreator, each string contains a name
     *                         of a block and his creator.
     */
    public BlocksFromSymbolsFactory(Map<String, Integer> spacerWidthsMap, Map<String, BlockCreator> blockCreatorsMap) {
        this.spacerWidths = spacerWidthsMap;
        this.blockCreators = blockCreatorsMap;
    }

    /**
     * @param s - the string for checking.
     * @return returns true if 's' is a valid space symbol. Otherwise - false.
     */
    public boolean isSpaceSymbol(String s) {
        return this.spacerWidths.containsKey(s);
    }

    /**
     * @param s - the string for checking.
     * @return true if 's' is a valid block symbol. Otherwise - false.
     */
    public boolean isBlockSymbol(String s) {
        return this.blockCreators.containsKey(s);
    }

    /**
     * @param s - the string for checking.
     * @return the width of this string called s.
     */
    public int getSpaceWidth(String s) {
        return this.spacerWidths.get(s);
    }

    /**
     * @param s - the symbol of the returning Block.
     * @param x - x position of this new block.
     * @param y - y position of this new block.
     * @return new block
     */
    public Block getBlock(String s, int x, int y) {
        return this.blockCreators.get(s).create(x, y);
    }

}
