import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.Reader;

/**
 * Tester.
 *
 * @author Ofir Ben Shoham
 */
public class TesterSomeLevels {

    /**
     * @param args none.
     */
    public static void main(String[] args) {

        Reader fileReader;

        try {
            fileReader = new FileReader("C:/Users/user/Downloads/levelsExample.txt");
            LevelSpecificationReader ls = new LevelSpecificationReader();
            ls.fromReader(fileReader);
        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

}
