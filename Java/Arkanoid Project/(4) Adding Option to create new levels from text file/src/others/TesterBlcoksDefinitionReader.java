import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Paths;

/**
 * Tester class.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-06-13
 */
public class TesterBlcoksDefinitionReader {

    /**
     * @param args - none.
     */
    public static void main(String[] args) {
        Reader fileReader;
        try {
            fileReader = new FileReader("C:/Users/user/Downloads/blockDefExample.txt");
            BlocksFromSymbolsFactory symbolsBlock = BlocksDefinitionReader.fromReader(fileReader);
            Block a1 = symbolsBlock.getBlock("z", 30, 30);
            a1.printBlock();

        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        try {
            StringBuilder str = new StringBuilder();
            // File f2 = new File("C:/Users/user/Downloads/checker.txt");
            for (String line : Files.readAllLines(Paths.get("C:/Users/user/Downloads/checker.txt"))) {
                str.append(line + "\n");
            }
            System.out.println("\n" + BlocksDefinitionReader.getMapSpacersDefinitions(str.toString()));
        } catch (IOException e) {
            System.out.println("Problem open this file");
            e.printStackTrace();
        }

        try {
            StringBuilder str2 = new StringBuilder();
            // File f2 = new File("C:/Users/user/Downloads/checker.txt");
            for (String line : Files.readAllLines(Paths.get("C:/Users/user/Downloads/defChecker.txt"))) {
                str2.append(line + "\n");
            }
            System.out.println("\n" + BlocksDefinitionReader.defaultsFromString(str2.toString()));
        } catch (IOException e) {
            System.out.println("Problem open this file");
            e.printStackTrace();
        }
    }
}
