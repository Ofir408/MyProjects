import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.LineNumberReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


/**
 * This program is responsible to read the sets levels from a file. For example
 * from a file like the following:
 * e:Easy definitions/easy_level_definitions.txt h:Hard
 * definitions/hard_level_definitions.txt
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-20
 */
public class SetsReaderOfLevels {

    /**
     * @param reader - reader to read from our file and it relatives to the
     *               class-path.
     * @return Map<String, List<LevelInformation>> for example:
     * Key is "e" and his value is list of easy levelInformations.
     */
    public static Map<String, List<LevelInformation>> setCreator(java.io.Reader reader) {

        Map<String, List<LevelInformation>> mapToReturn = new HashMap<>();
        BufferedReader buffReader = null;
        String currentLine, levelSymbol = "";
        int lineNumber = 0;

        try {
            LineNumberReader lineNumberReader = new LineNumberReader(reader);
            while ((currentLine = lineNumberReader.readLine()) != null) {
                //lineNumber = lineNumberReader.getLineNumber();
                if (currentLine.matches("(?m)^[ \t]*\r?\n") || currentLine.matches("")) {
                    continue;
                }

                lineNumber++;

                if ((lineNumber % 2 == 1)) {
                    String[] s = currentLine.split(":");
                    levelSymbol = s[0];
                } else {

                    InputStreamReader in = new InputStreamReader(ClassLoader.getSystemClassLoader()
                            .getResourceAsStream(currentLine));

                    buffReader = new BufferedReader(in);
                    mapToReturn.put(levelSymbol, LevelSpecificationReader.fromReader(buffReader));
                }
            }
            return mapToReturn;
        } catch (IOException io) {
            System.out.println("I had a problem trying parsing the level sets");
            return new HashMap<>(); // empty map returned in this case.
        } catch (NullPointerException n) {
            System.out.println(" NullPointerException problem ");
            n.printStackTrace();
            return new HashMap<>(); // empty map returned in this case.
        }
    }


    /**
     * @param symbol - a string (symbol) for the list of levels.
     * @param reader - to read from a file.
     * @return List<LevelInformation> of the input symbol in our map - if have.
     * Otherwise - return empty list (if didn't find the symbol in the map).
     */
    public static List<LevelInformation> getLevelsOfSymbol(String symbol, java.io.Reader reader) {
        Map<String, List<LevelInformation>> mapForChecking =
                SetsReaderOfLevels.setCreator(reader);
        if (mapForChecking.containsKey(symbol)) {
            return mapForChecking.get(symbol);
        } else {
            System.out.println("I didn't find this symbol: " + symbol);
            return new ArrayList<>(); // empty list was returned.
        }
    }

    /**
     * @param reader - reader with the text.
     * @return Map<String, String> -> symbols to messages to show.
     */
    public static Map<String, String> getSymbolsAndMessages(java.io.Reader reader) {
        Map<String, String> mapToRetutn = new HashMap<>();
        int lineNumber = 0;
        try {
            String currentLine;
            LineNumberReader lineReader = new LineNumberReader(reader);
            while ((currentLine = lineReader.readLine()) != null) {
                if (currentLine.matches("(?m)^[ \t]*\r?\n") || currentLine.matches("")) {
                    continue;
                }
                lineNumber++;

                if ((lineNumber % 2 == 1)) {
                    String[] s = currentLine.split(":");
                    mapToRetutn.put(s[0], s[1]);
                }
            }
        } catch (IOException io) {
            System.out.println("There is a problem in getSymbolsAndMessages function ");
            return null;
        }
        return mapToRetutn;
    }
}
