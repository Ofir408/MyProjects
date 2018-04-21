import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.StringReader;
import java.util.HashMap;
import java.util.Map;

/**
 * BlocksDefinitionReader Class that read the block difiinition file and return
 * us a map of block creators.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-06-13
 */
public class BlocksDefinitionReader {

    /**
     * @param reader - reader for the text file.
     * @return BlocksFromSymbolsFactory object.
     */
    public static BlocksFromSymbolsFactory fromReader(java.io.Reader reader) {
        Map<String, String> defaults = new HashMap<>();
        Map<String, BlockCreator> blockCreatorsMap = new HashMap<>();
        Map<String, Integer> spacerWidths = new HashMap<>();

        StringBuffer fileData = new StringBuffer();
        char[] buf = new char[1024];
        int character;
        try {
            while ((character = reader.read(buf)) != -1) {
                String readData = String.valueOf(buf, 0, character);
                fileData.append(readData);
            }
            reader.close();
        } catch (IOException e) {
            System.out.println("There is a problem with the reader");
        }
        String[] lines = fileData.toString().split("\n");

        for (int i = 0; i < lines.length; i += 1) {
            // for each block definition.
            if (lines[i].contains("default")) {
                defaults = BlocksDefinitionReader.defaultsFromString(lines[i]);
            }
            if (lines[i].contains("bdef")) {
                blockCreatorsMap.putAll(BlocksDefinitionReader.
                        praseLineOfBlockDefiniton(defaults, lines[i]));
            }
            if (lines[i].contains("sdef")) {
                spacerWidths.putAll(BlocksDefinitionReader.getMapSpacersDefinitions(lines[i]));
            }
        } // end passing all the lines in our file.
        return new BlocksFromSymbolsFactory(spacerWidths, blockCreatorsMap);
    }

    /**
     * @param s - string to remove his remarks from and return new string Note
     *          that s does not change in this function.
     * @return new string as s just without remarks For example, should remove
     * the lines: # default values for blocks And # block definitions
     */
    public static String removeRemarks(String s) {
        BufferedReader bufReader = new BufferedReader(new StringReader(s));
        StringBuilder newString = new StringBuilder();
        String line = null;
        try {
            while ((line = bufReader.readLine()) != null) {
                if (!line.startsWith("#") && !line.isEmpty()) {
                    String lineWithLineBreak = line.toString() + "\n";
                    newString.append(lineWithLineBreak);
                }
            }
            return newString.toString();
        } catch (IOException e) {
            System.out.println("IO exception in bufReader iterates the lines");
            System.out.println("There was a problem, return null");
            return null;
        }
    }

    /**
     * @param s - the string that represents the text file. Handle with line
     *          as: " default height:25 width:50 stroke:color(black)
     *          hit_points:1 ".
     * @return Map<String, String> between the default name - a string and his
     * value, for example: Key: width, Value 50 or Key: stroke Value
     * color(black) and more...
     */
    public static Map<String, String> defaultsFromString(String s) {
        Map<String, String> defaultsMap = new HashMap<>();
        if (!s.contains("default")) {
            System.out.println("You try to bring a defaults Map " + "from a line who is not for defaults");
            return defaultsMap;
        }
        String withoutRemarks = BlocksDefinitionReader.removeRemarks(s);
        String withoutEmptyLines = withoutRemarks.replaceAll("([\\n\\r]+\\s*)*$", "");

        String[] dividedStrings = withoutEmptyLines.split(" ");
        for (int i = 0; i < dividedStrings.length; i++) {
            String t = dividedStrings[i];
            defaultsMap.putAll(BlocksDefinitionReader.helperWithDefaults(t));
        }
        return defaultsMap;
    }

    /**
     * @param s - string as: height:25 or width:50 or stroke:color(black) or
     *          hit_points:1
     * @return new Map<String, String> that represents one default. For example:
     * key: height, value: 25 or key: stroke, value: block.
     */
    private static Map<String, String> helperWithDefaults(String s) {
        Map<String, String> mapWithOneDefault = new HashMap<>();
        String withOutSpaces = s.replaceAll("\\s+", ""); // remove spaces.
        String[] newString = withOutSpaces.split(":");
        try {
            mapWithOneDefault.put(newString[0], newString[1]);
            return mapWithOneDefault;
        } catch (Exception e) {
            return mapWithOneDefault; // an empty map was returned.
        }
    }

    /**
     * @param s - string. For example, strings as: symbol:- width:30 or
     *          symbol:* width:20 and more...
     * @return new Map <String, Integer>, for example if the input string is:
     * sdef symbol:- width:30 so the returned map should be as the
     * following: Key: "-" and his Value should be: "30".
     */
    public static Map<String, Integer> getMapSpacersDefinitions(String s) {
        Map<String, Integer> spacersMap = new HashMap<>();

        String withoutRemarks = BlocksDefinitionReader.removeRemarks(s);
        String withoutEmptyLines = withoutRemarks.replaceAll("([\\n\\r]+\\s*)*$", "");
        try {
            if (!s.contains("sdef")) {
                return spacersMap; // empty map returned.
            } else {
                String withoutSdef = withoutEmptyLines.replaceAll("sdef ", "");
                String[] parts = withoutSdef.split(" ");
                String[] firstNeeded = parts[0].split(":");
                String[] secondNeeded = parts[1].split(":");
                spacersMap.put(firstNeeded[1], Integer.parseInt(secondNeeded[1]));
                return spacersMap;
            }
        } catch (Exception e) {
            return spacersMap; // empty map was returned.
        }
    }

    /**
     * @param defaultMap  - Map<String, String> with all the defaults.
     * @param currentLine - String to parse, of definition of block.
     * @return Map<String, BlockCreator>
     */
    public static Map<String, BlockCreator> praseLineOfBlockDefiniton(Map<String, String> defaultMap,
                                                                      String currentLine) {
        Map<String, BlockCreator> mapToReturn = new HashMap<>();
        if (!currentLine.contains("bdef")) {
            return mapToReturn;
        }
        int hitPointNumber = 0;
        double height = 0, width = 0;
        Color strokeColor = null, blockColorIfHave = null;
        Map<Integer, Color> colorsHitsMap = new HashMap<>();
        Map<Integer, File> fileImagesHitsMap = new HashMap<>();
        char blockSymbol = 0;
        File imageFilePath = null;

        // start checking
        String withoutBdef = currentLine.replaceAll("bdef ", "");
        String[] t = withoutBdef.split(" ");
        try {
            for (int i = 0; i < t.length; i++) {
                String[] eachOne = t[i].split(":", 2);
                if (eachOne[0].equals("symbol")) {
                    if (eachOne[1].length() != 1 || !eachOne[1].toLowerCase().matches(".*[a-z].*")) {
                        System.out.println("Symbol block is not valid");
                        System.exit(0);
                    } else {
                        // valid
                        blockSymbol = eachOne[1].charAt(0);
                    }
                }
                if (eachOne[0].equals("width")) {
                    int temp = (int) Double.parseDouble(eachOne[1]);
                    if (temp <= 0) {
                        System.out.println("width is not positive integer");
                        System.exit(0);
                    } else {
                        // width is valid
                        width = Double.parseDouble(eachOne[1]);
                    }
                }
                if (eachOne[0].equals("hit_points")) {
                    if (!eachOne[1].matches("[1-9]\\d*")) {
                        System.out.println("hit_points is not positive integer");
                        System.exit(0);
                    } else {
                        hitPointNumber = Integer.parseInt(eachOne[1]);
                    }
                }
                if (eachOne[0].equals("height")) {
                    int temp = (int) Double.parseDouble(eachOne[1]);
                    if (temp <= 0) {
                        System.out.println("height is not positive integer");
                        System.exit(0);
                    } else {
                        height = Double.parseDouble(eachOne[1]);
                    }
                }
                if (eachOne[0].equals("stroke")) {
                    if (eachOne[1].contains("color")) {
                        try {
                            ColorsParser cp = new ColorsParser();
                            strokeColor = cp.colorFromString(eachOne[1]);
                        } catch (Exception e) {
                            strokeColor = null;
                        }
                    } // else - defaults, if no default -> no border should be
                    // drawn.
                }
                // add if , for color map and image map, if have..
                if (eachOne[0].equals("fill")) {
                    if (eachOne[1].contains("color")) {
                        try {
                            ColorsParser cp = new ColorsParser();
                            blockColorIfHave = cp.colorFromString(eachOne[1]);
                        } catch (Exception e) {
                            blockColorIfHave = null;
                        }
                    } else {
                        // check if need to fill an image
                        if (eachOne[1].contains("image")) {
                            String temp = eachOne[1];
                            String t1 = temp.replace("image(", "");
                            String t2 = t1.replace(")", "");
                            if (t2.contains(".")) {
                                // more checker, to verify
                                imageFilePath = new File(t2);
                            }
                        }
                    }
                }
                // fill-2:image(block_images/leopard.jpg)
                if (eachOne[0].contains("fill-")) {
                    String[] findFillNumber = eachOne[0].split("-");
                    try {
                        int number = Integer.parseInt(findFillNumber[1]);
                        if (eachOne[1].contains("image")) {
                            String temp = eachOne[1];
                            temp = temp.replace("image(", "");
                            temp = temp.replace(")", "");
                            if (temp.contains(".")) {
                                // more checker, to verify
                                File currentFile = new File(temp);
                                fileImagesHitsMap.put(number, currentFile);
                            }
                        }
                        // fill-3:color(RGB(244,248,129))
                        if (eachOne[1].contains("color")) {
                            try {
                                ColorsParser cp = new ColorsParser();
                                Color currentColor = cp.colorFromString(eachOne[1]);
                                colorsHitsMap.put(number, currentColor);
                            } catch (Exception e) {
                                continue;
                            }
                        }

                    } catch (NumberFormatException n) {
                        continue;
                    }
                }
            } // end of for loop
            if (hitPointNumber > 1) {
                for (int i = 1; i < hitPointNumber + 1; i++) {
                    if (blockColorIfHave != null && !colorsHitsMap.containsKey(i)) {
                        colorsHitsMap.put(i, blockColorIfHave);
                    } else if (imageFilePath != null && !fileImagesHitsMap.containsKey(i)) {
                        fileImagesHitsMap.put(i, imageFilePath);
                    }
                }
            }

        } catch (Exception e) {
            System.out.println("empty map was returned ");
            e.printStackTrace();
            return mapToReturn;
        }
        BlockFileFields filedsObjcet = new BlockFileFields(width, hitPointNumber, imageFilePath, strokeColor,
                blockColorIfHave, colorsHitsMap, fileImagesHitsMap);
        filedsObjcet.setHeight(height);
        BlockFileFields afterEnteringDefaults = BlocksDefinitionReader.enterDefaults(filedsObjcet, defaultMap);
        if (blockSymbol != 0) {
            return BlocksDefinitionReader.getFinalCreators(blockSymbol, afterEnteringDefaults);
        }
        return new HashMap<>();
    }

    /**
     *
     * @param fieldsObj - BlockFileFields with all the data about the block.
     * @param defaultMap - defaults.
     * @return BlockFileFields after entering the defaults.
     */
    private static BlockFileFields enterDefaults(BlockFileFields fieldsObj, Map<String, String> defaultMap) {
        if (!fieldsObj.hasHeight()) {
            if (defaultMap.containsKey("height")) {
                fieldsObj.setHeight(Double.parseDouble(defaultMap.get("height")));
            } else {
                System.out.println("Problem in prasing, no height value");
            }
        }
        if (!fieldsObj.hasWidth()) {
            if (defaultMap.containsKey("width")) {
                fieldsObj.setWidth(Double.parseDouble(defaultMap.get("width")));
            } else {
                System.out.println("Problem in prasing, no height");
            }
        }
        if (!fieldsObj.hasHitPoints()) {
            if (defaultMap.containsKey("hit_points")) {
                fieldsObj.setHitPoints(Integer.parseInt(defaultMap.get("hit_points")));
            } else {
                System.out.println("Problem in prasing, no hit_points");
            }
        }

        if (!fieldsObj.hasHitPoints() || !fieldsObj.hasWidth() || !fieldsObj.hasHeight()) {
            System.out.println("Prasing problem, no enough values");
            System.exit(0);
        }
        if (!fieldsObj.hasStroke() && defaultMap.containsKey("stroke")) {
            fieldsObj.setStrokeColor(new ColorsParser().colorFromString(defaultMap.get("stroke")));
        }
        if (!fieldsObj.hasBlockColor() && defaultMap.containsKey("fill")) {
            if (defaultMap.get("fill").contains("color")) {
                fieldsObj.setBlockColor(new ColorsParser().colorFromString(defaultMap.get("fill")));
            } else if (defaultMap.get("fill").contains("image")) {
                fieldsObj.setImageFilePath(new File(defaultMap.get("fill")));
            }
        }
        if (!fieldsObj.hasBlockColor() && !fieldsObj.hasOneImage() && !fieldsObj.hasFilesMap()
                && !fieldsObj.hasColorMap()) {
            System.out.println("no enough values, Prasing failed");
            System.exit(0);
        }
        return fieldsObj;
    }

    /**
     * This is for one object - one line of block definition.
     *
     * @param symbol - the symbol (char) of this block.
     * @param b      - object with all the needed fields.
     * @return Map<String, BlockCreator>
     */
    private static Map<String, BlockCreator> getFinalCreators(char symbol, BlockFileFields b) {
        Map<String, BlockCreator> mapToReturn = new HashMap<>();

        if (b.hasColorMap() && b.hasFilesMap()) {
            mapToReturn.put(symbol + "", new BlockCreater(b.getHeight(), b.getWidth(), b.getHitPoints(),
                    b.getMapFileHitPoints(), b.getMapColorHitPoints(), b.getStrokeColor()));
        } else if (b.hasColorMap()) {
            mapToReturn.put(symbol + "", new BlockCreater(b.getHeight(), b.getWidth(), b.getHitPoints(),
                    b.getStrokeColor(), b.getMapColorHitPoints()));
        } else if (b.hasFilesMap() && !b.getMapFileHitPoints().isEmpty()) {
            mapToReturn.put(symbol + "", new BlockCreater(b.getHeight(), b.getWidth(), b.getHitPoints(),
                    b.getMapFileHitPoints(), b.getStrokeColor()));
        } else if (b.hasBlockColor()) {
            mapToReturn.put(symbol + "", new BlockCreater(b.getHeight(), b.getWidth(), b.getHitPoints(),
                    b.getStrokeColor(), b.getBlockColor()));
        } else {
            mapToReturn.put(symbol + "", new BlockCreater(b.getHeight(), b.getWidth(), b.getHitPoints(),
                    b.getStrokeColor(), b.getImageFilePath()));
        }

        return mapToReturn;
    }
}
