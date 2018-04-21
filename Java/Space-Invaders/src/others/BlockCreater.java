import java.awt.Color;
import java.io.File;
import java.util.Map;

/**
 * BlockCreater Class that has constructors for block, This class Implement.
 * Block creator
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-13
 */
public class BlockCreater implements BlockCreator {

    private double height, width;
    private int hitPoints;
    private File imageFilePath;
    private Color strokeColor, blockColor;
    private Map<Integer, Color> mapColorHitPoints;
    private Map<Integer, File> mapFileHitPoints;

    /**
     * @param blockHeight    - height of the block.
     * @param blockWidth     - width of the block.
     * @param hitPointNumber - hit points number of the block.
     * @param newStrokeColor - the color of the block stroke.
     */
    public BlockCreater(double blockHeight, double blockWidth, int hitPointNumber,
                        Color newStrokeColor) {
        this.height = blockHeight;
        this.width = blockWidth;
        this.hitPoints = hitPointNumber;
        this.strokeColor = newStrokeColor;
    }

    /**
     * @param blockHeight    - height of the block.
     * @param blockWidth     - width of the block.
     * @param hitPointNumber - hit points number of the block.
     * @param newStrokeColor - the color of the block stroke.
     * @param newBlockColor  - Color of the block.
     */
    public BlockCreater(double blockHeight, double blockWidth, int hitPointNumber,
                        Color newStrokeColor, Color newBlockColor) {
        this(blockHeight, blockWidth, hitPointNumber, newStrokeColor);
        this.blockColor = newBlockColor;
    }

    /**
     * @param blockHeight    - height of the block.
     * @param blockWidth     - width of the block.
     * @param hitPointNumber - hit points number of the block.
     * @param newStrokeColor - the color of the block stroke.
     * @param imageFile      - image file path, this image is for the background
     *                       on the block.
     */
    public BlockCreater(double blockHeight, double blockWidth, int hitPointNumber,
                        Color newStrokeColor, File imageFile) {
        this(blockHeight, blockWidth, hitPointNumber, newStrokeColor);
        this.imageFilePath = imageFile;
    }

    /**
     * @param blockHeight           - height of the block.
     * @param blockWidth            - width of the block.
     * @param hitPointNumber        - hit points number of the block.
     * @param newStrokeColor        - the color of the block stroke.
     * @param newMapHitPointsColors - Map<Integer, Color>, for each number of hitPoints
     *                              there is a color to draw on the block.
     */
    public BlockCreater(double blockHeight, double blockWidth, int hitPointNumber,
                        Color newStrokeColor, Map<Integer, Color> newMapHitPointsColors) {
        this(blockHeight, blockWidth, hitPointNumber, newStrokeColor);
        this.mapColorHitPoints = newMapHitPointsColors;
    }

    /**
     * @param blockHeight            - height of the block.
     * @param blockWidth             - width of the block.
     * @param hitPointNumber         - hit points number of the block.
     * @param newStrokeColor         - the color of the block stroke.
     * @param imageFilesForHitPoints - Map<Integer, File>, for each number of hitPoints
     *                               there is file path that has image to draw on the block.
     */

    public BlockCreater(double blockHeight, double blockWidth, int hitPointNumber,
                        Map<Integer, File> imageFilesForHitPoints, Color newStrokeColor) {
        this(blockHeight, blockWidth, hitPointNumber, newStrokeColor);
        this.mapFileHitPoints = imageFilesForHitPoints;
    }

    /**
     * @param blockHeight            - height of the block.
     * @param blockWidth             - width of the block.
     * @param hitPointNumber         - hit points number of the block.
     * @param newStrokeColor         - the color of the block stroke.
     * @param imageFilesForHitPoints - Map<Integer, File>.
     * @param colorsMap -  Map<Integer, Color>.
     */
    public BlockCreater(double blockHeight, double blockWidth, int hitPointNumber,
                        Map<Integer, File> imageFilesForHitPoints, Map<Integer, Color> colorsMap,
                        Color newStrokeColor) {
        this(blockHeight, blockWidth, hitPointNumber, newStrokeColor);
        this.mapFileHitPoints = imageFilesForHitPoints;
        this.mapColorHitPoints = colorsMap;
    }

    /**
     * @param x         - x position of the rectangle.
     * @param y         -  y position of the rectangle.
     * @param newWidth  - width of the rectangle.
     * @param newHeight - height of the rectangle.
     * @return new rectangle with this features.
     */
    private static Rectangle rectangleBulider(int x, int y, double newWidth, double newHeight) {
        Rectangle r = new Rectangle(new Point(x, y), newWidth, newHeight);
        return r;
    }

    @Override
    public Block create(int xpos, int ypos) {
        Rectangle r = BlockCreater.rectangleBulider(xpos, ypos, width, height);

        // if both colors  and both images maps
        if (this.mapColorHitPoints != null && this.mapFileHitPoints != null) {
            return new Block(r, hitPoints, strokeColor, mapColorHitPoints, mapFileHitPoints);
        }
        if (this.imageFilePath != null) {
            return new Block(r, hitPoints, imageFilePath, strokeColor);
        }
        if (this.blockColor != null) {
            return new Block(r, hitPoints, blockColor, strokeColor);
        }
        if (this.mapFileHitPoints != null) {
            return new Block(r, hitPoints, mapFileHitPoints, strokeColor);
        }
        return new Block(r, hitPoints, strokeColor, mapColorHitPoints);
    }

}
