import java.awt.Color;
import java.awt.Image;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;

/**
 * LevelSpecificationReader class.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-06-17
 */
public class LevelSpecificationReader {

    /**
     * @param reader - reader to read the file.
     * @return List<LevelInformation>.
     */
    public static List<LevelInformation> fromReader(java.io.Reader reader) {
        List<LevelInformation> levelInfoList = new ArrayList<>();
        try {
            StringBuffer fileData = new StringBuffer();
            char[] buf = new char[2048];
            int character;
            while ((character = reader.read(buf)) != -1) {
                String readData = String.valueOf(buf, 0, character);
                fileData.append(readData);
            }
            reader.close();

            List<String> levels = splitForLevels(fileData.toString());
            if (levels.size() == 1) {
                levelInfoList.add(levelParser(levels.get(0)));
            }
            for (int index = 0; index < levels.size(); index++) {
                levelInfoList.add(levelParser(levels.get(index)));
            }
            return levelInfoList;

        } catch (IOException e) {
            e.printStackTrace();
        }

        return null;
    }

    // helper functions

    /**
     * @param fullText - the full text with all the levels.
     * @return List<String> of levels.
     */
    public static List<String> splitForLevels(String fullText) {
        String[] s = fullText.split("END_LEVEL");
        List<String> levels = new ArrayList<>();
        for (int i = 0; i < s.length; i += 1) {
            if (!s[i].matches("(?m)^[ \t]*\r?\n")) {
                // without empty levels.
                levels.add(s[i]);
            }
        }
        // for (int h=0; h<levels.size();h++) {
        // this.levelParser(levels.get(0));
        // }
        return levels;
    }

    /**
     * @param fullLevel - string to get subString from.
     * @return Substring Of just Blocks String.
     */
    public static String substringOfBlocks(String fullLevel) {
        String[] lines = fullLevel.split("\n");
        StringBuilder s = new StringBuilder();
        boolean t = false;
        for (int i = 0; i < lines.length; i++) {
            if (t) {
                s.append(lines[i] + "\n");
            }
            if (lines[i].contains("START_BLOCKS")) {
                t = true;
            }
            if (lines[i].contains("END_LEVEL")) {
                t = false;
            }
        }
        return s.toString();
    }

    /**
     * @param levelString - string of level to parse.
     * @return LevelInformation.
     */
    public static LevelInformation levelParser(String levelString) {

        String fullLevelString = levelString;
        String[] lines = levelString.split("\n");
        String levelName = "";
        List<Velocity> ballVelocities = null;
        BackgroundFromFile background = null;
        int paddleSpeed = 0, paddleWidth = 0, blocksStartX = 0, blocksStartY = 0, rowHeight = 0, numBlocks = 0;
        File blockDefinitinonsFile = null;
        BlocksFromSymbolsFactory factory = null;

        for (int i = 0; i < lines.length; i += 1) {
            String currentLine = lines[i];
            String[] eachOne = currentLine.split(":");

            try {
                // String oneFromEachOne = eachOne[1].trim();
                if (eachOne[0].equals("level_name") && !eachOne[1].isEmpty()) {
                    levelName = eachOne[1];
                }
                if (eachOne[0].equals("ball_velocities")) {
                    ballVelocities =
                            LevelSpecificationReader.fromStringToVelocities(eachOne[1]);
                }
                if (eachOne[0].equals("background")) {
                    if (eachOne[1].contains("image")) {
                        // background:image(C:/Users/user/Downloads/zebra.jpg)
                        String[] sp = currentLine.split(":", 2);
                        String sub = sp[1].trim().replace("image(", "");
                        String finalSub = sub.replace(")", "");
                        // String[] sub = currentLine.split("(");

                        // trying here convert to image.
                        try {
                            Path currentRelativePath = Paths.get("");
                            String s = currentRelativePath.toAbsolutePath().toString().concat("/resources/")
                                    .concat(finalSub);
                            Image img2 = ImageIO.read(new File(s));

                            background = new BackgroundFromFile(img2);
                        } catch (Exception e) {
                            System.out.println("didn't convert to image in LevelSpecReader (1)");
                        }

                    } else if (eachOne[1].contains("color")) {
                        ColorsParser cp = new ColorsParser();
                        background = new BackgroundFromFile(cp.colorFromString(eachOne[1]));
                    }
                }
                if (eachOne[0].equals("paddle_speed")) {
                    // paddleSpeed = (int) LevelSpecificationReader.padSpeed(Integer.parseInt(eachOne[1].trim()));
                    paddleSpeed = Integer.parseInt(eachOne[1].trim());
                }
                if (eachOne[0].equals("paddle_width")) {
                    paddleWidth = Integer.parseInt(eachOne[1].trim());
                }
                if (eachOne[0].equals("block_definitions")) {
                    // call to helper function that return List<Block> from a
                    // file
                    String[] sp = currentLine.split(":", 2);
                    blockDefinitinonsFile = new File(sp[1]);
                    try {
                        BufferedReader br = new BufferedReader(new InputStreamReader(ClassLoader.getSystemClassLoader()
                                .getResourceAsStream((blockDefinitinonsFile.toString().trim()))));
                        factory = BlocksDefinitionReader.fromReader(br);

                    } catch (Exception e) {
                        System.out.println("problem to get BlocksFromSymbolsFactory factory\n");
                        System.out.println("Ended with a problem & System.exit(0)");
                        System.exit(0);
                        factory = null;
                    }
                }
                if (eachOne[0].equals("blocks_start_x")) {
                    blocksStartX = Integer.parseInt(eachOne[1].trim());
                }
                if (eachOne[0].equals("blocks_start_y")) {
                    blocksStartY = Integer.parseInt(eachOne[1].trim());
                }
                if (eachOne[0].equals("row_height")) {
                    rowHeight = Integer.parseInt(eachOne[1].trim());
                }
                if (eachOne[0].equals("num_blocks")) {
                    numBlocks = Integer.parseInt(eachOne[1].trim());
                }
            } catch (NumberFormatException n) {
                System.out.println("Problem to convert from file to levlInformation");
            }
        }
        if (levelName.length() == 0 || ballVelocities == null || background == null || paddleSpeed == 0
                || paddleWidth == 0 || blockDefinitinonsFile == null || blocksStartX == 0 || blocksStartY == 0
                || rowHeight == 0 || numBlocks == 0) {
            System.out.println("no enough data to parsing to level information");
            System.exit(0);
        }

        // get final blocks list of this level.
        List<Block> blocksList = LevelSpecificationReader.getBlocksFromString(fullLevelString, factory, blocksStartX,
                blocksStartY, rowHeight);

        Color backColor = background.getColor();
        if (backColor != null) {
            return new LevelFileCreator(levelName, ballVelocities, paddleWidth, paddleSpeed, numBlocks, blocksList,
                    backColor);
        }
        Image img = background.getImage();
        if (img == null) {
            System.out.println("There is a problem in LevelSpecificationReader");
            // System.exit(0);
        }
        return new LevelFileCreator(levelName, ballVelocities, paddleWidth, paddleSpeed, numBlocks, blocksList, img);
    }

    /**
     * @param velString - string with data about ball_velocities.
     * @return List<Velocity> with the velocities from the input string.
     */
    private static List<Velocity> fromStringToVelocities(String velString) {
        List<Velocity> velList = new ArrayList<>();
        double speed, angle;
        String[] velDiffrentParts = velString.split(" ");
        for (int i = 0; i < velDiffrentParts.length; i++) {
            String[] twoPartsOfEachVel = velDiffrentParts[i].split(",");
            try {
                angle = Double.parseDouble(twoPartsOfEachVel[0]);
                speed = Double.parseDouble(twoPartsOfEachVel[1]);
                velList.add(Velocity.fromAngleAndSpeed(angle, speed));
            } catch (NumberFormatException n) {
                System.out.println(" A problem in parsing In fromStringToVelocities method");
                continue;
            }
        }
        return velList;
    }

    /**
     * @param vel - Velocity list.
     * @return list of velocities.
     */
    private static List<Velocity> velHelper(List<Velocity> vel) {
        return vel;
    }

    /**
     * No use in this function...
     *
     * @param current - current velocity
     * @return new velocity value.
     */
    private static double padSpeed(double current) {
        if (current > 600) {
            return current / 35;
        }
        if (current > 500) {
            return current / 30;
        }
        if (current > 400) {
            return current / 25;
        }
        if (current > 300) {
            return current / 20;
        }
        if (current > 200) {
            return current / 15;
        }
        if (current > 100) {
            return current / 10;
        }
        if (current > 20) {
            return current / 7.5;
        }
        return current;
    }

    /**
     * @param s           - string to get the blocks from.
     * @param factory     - BlocksFromSymbolsFactory has constructors of this blocks.
     * @param blockStartX - x point.
     * @param blockStartY - y point.
     * @param rowHeight   - the height of the row.
     * @return List<Block>- list of blocks from the input string.
     */
    private static List<Block> getBlocksFromString(String s, BlocksFromSymbolsFactory factory, int blockStartX,
                                                   int blockStartY, int rowHeight) {

        String justBlocksDef = LevelSpecificationReader.substringOfBlocks(s);
        double width = 0;
        int xBeforeChanges = blockStartX;
        List<String> linesAfterDelete = new ArrayList<>();
        List<Block> finalBlocksList = new ArrayList<>();

        String[] lines = justBlocksDef.split("\n");
        for (String l : lines) {
            if (!l.contains("START_BLOCKS") && !l.contains("END_BLOCKS")) {
                String temp = l.replaceAll("([\\n\\r]+\\s*)*$", "");
                linesAfterDelete.add(temp);
            }
        }
        String[] afterDelete = new String[linesAfterDelete.size()];
        for (int i = 0; i < linesAfterDelete.size(); i++) {
            afterDelete[i] = linesAfterDelete.get(i);
        } // now the string in afterDelete array of strings.

        for (int i = 0; i < afterDelete.length; i++) {
            boolean bl = true;
            for (char c : afterDelete[i].toCharArray()) {
                if (factory.isBlockSymbol(c + "")) {
                    Block newBlock = factory.getBlock(c + "", blockStartX, blockStartY);
                    finalBlocksList.add(newBlock);
                    width = newBlock.getCollisionRectangle().getWidth();
                    blockStartX = (blockStartX + (int) width);
                    bl = false;
                }
                if (factory.isSpaceSymbol(c + "")) {
                    blockStartX += factory.getSpaceWidth(c + "");
                    bl = false;
                }
            }
            if (!bl) {
                blockStartY = blockStartY + rowHeight;
                blockStartX = xBeforeChanges;
            }
        }
        return finalBlocksList;
    }
}
