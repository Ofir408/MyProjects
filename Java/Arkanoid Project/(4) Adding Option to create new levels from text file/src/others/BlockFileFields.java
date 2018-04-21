import java.awt.Color;
import java.io.File;
import java.util.Map;

/**
 * That's BlockFileFields class. This class have all the optional fields of the
 * block we read from Block definition file.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-04-27
 */

public class BlockFileFields {

    private double height, width;
    private int hitPoints;
    private File imageFilePath;
    private Color strokeColor, blockColor;
    private Map<Integer, Color> mapColorHitPoints;
    private Map<Integer, File> mapFileHitPoints;

    /**
     * @param w               - width.
     * @param hitPointsNumber - number of hit points.
     * @param imagePath       - path of file.
     * @param stroke          - color of the stroke of the ball.
     * @param blockCol        - color of the block.
     * @param colorsMap       - Map<Integer, Color> colorsMap.
     * @param imagesFilesMap  - Map<Integer, File> imagesFilesMap.
     */
    public BlockFileFields(double w, int hitPointsNumber, File imagePath, Color stroke, Color blockCol,
                           Map<Integer, Color> colorsMap, Map<Integer, File> imagesFilesMap) {
        this.width = w;
        this.hitPoints = hitPointsNumber;
        this.imageFilePath = imagePath;
        this.strokeColor = stroke;
        this.blockColor = blockCol;
        this.mapColorHitPoints = colorsMap;
        this.mapFileHitPoints = imagesFilesMap;
    }

    /**
     * @return the height of this block.
     */
    public double getHeight() {
        return height;
    }

    /**
     * @param newHeight - height to set.
     */
    public void setHeight(double newHeight) {
        this.height = newHeight;
    }

    /**
     * @return the getWidth of this block.
     */
    public double getWidth() {
        return width;
    }

    /**
     * @param newWidth - newEidth to set.
     */
    public void setWidth(double newWidth) {
        this.width = newWidth;
    }

    /**
     * @return HitPoints number.
     */
    public int getHitPoints() {
        return hitPoints;
    }

    /**
     * @param hitPointsNumber - hit points number to set.
     */
    public void setHitPoints(int hitPointsNumber) {
        this.hitPoints = hitPointsNumber;
    }

    /**
     * @return the image path file
     */
    public File getImageFilePath() {
        return imageFilePath;
    }

    /**
     * @param imageFile - file path to set.
     */
    public void setImageFilePath(File imageFile) {
        this.imageFilePath = imageFile;
    }

    /**
     * @return the stroke color.
     */
    public Color getStrokeColor() {
        return strokeColor;
    }

    /**
     * @param newStrokeColor - new Stroke Color.
     */
    public void setStrokeColor(Color newStrokeColor) {
        this.strokeColor = newStrokeColor;
    }

    /**
     * @return BlockColor.
     */
    public Color getBlockColor() {
        return blockColor;
    }

    /**
     * @param newBlockColor - new Block color to set.
     */
    public void setBlockColor(Color newBlockColor) {
        this.blockColor = newBlockColor;
    }

    /**
     * @return Map<Integer, Color> - MapColorHitPoints.
     */
    public Map<Integer, Color> getMapColorHitPoints() {
        return mapColorHitPoints;
    }

    /**
     * @param newMapColorHitPoints - Map<Integer, Color> - MapColorHitPoints.
     */
    public void setMapColorHitPoints(Map<Integer, Color> newMapColorHitPoints) {
        this.mapColorHitPoints = newMapColorHitPoints;
    }

    /**
     * @return MapFileHitPoints - Map<Integer, File>.
     */
    public Map<Integer, File> getMapFileHitPoints() {
        return mapFileHitPoints;
    }

    /**
     * @param newMapFileHitPoints - Map<Integer, File>.
     */
    public void setMapFileHitPoints(Map<Integer, File> newMapFileHitPoints) {
        this.mapFileHitPoints = newMapFileHitPoints;
    }

    /**
     * @return true if height != 0.
     */
    public boolean hasHeight() {
        return this.getHeight() != 0;
    }

    /**
     * @return true if Width != 0.
     */
    public boolean hasWidth() {
        return this.getWidth() != 0;
    }

    /**
     * @return true if HitPoints != 0.
     */
    public boolean hasHitPoints() {
        return this.getHitPoints() != 0;
    }

    /**
     * @return true if image path file != null.
     */
    public boolean hasOneImage() {
        return this.getImageFilePath() != null;
    }

    /**
     * @return true if hasStroke != null.
     */
    public boolean hasStroke() {
        return this.getStrokeColor() != null;
    }

    /**
     * @return true if blockColor != null.
     */
    public boolean hasBlockColor() {
        return this.getBlockColor() != null;
    }

    /**
     * @return true if mapColorHitPoints != null.
     */
    public boolean hasColorMap() {
        return this.mapColorHitPoints != null && !this.mapColorHitPoints.isEmpty();
    }

    /**
     * @return true if mapFileHitPoints != null.
     */
    public boolean hasFilesMap() {
        return this.mapFileHitPoints != null && !this.mapFileHitPoints.isEmpty();
    }
}
