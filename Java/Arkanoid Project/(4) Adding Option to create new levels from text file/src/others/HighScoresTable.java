import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * HighScoresTable class to handle with scores table.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-06
 */
public class HighScoresTable implements Serializable {

    /**
     *
     */
    private static final long serialVersionUID = 1L;
    private int tableSize;
    private List<ScoreInfo> allScoresInfo;

    /**
     * Create an empty high-scores table with the specified size. The size means
     * that the table holds up to size top scores. # create list of scoresInfo
     * interface to handle with the table score.
     *
     * @param size - maximum scores number in this table.
     */
    public HighScoresTable(int size) {
        this.tableSize = size;
        this.allScoresInfo = new ArrayList<>();
    }

    /**
     * Add a high-score.
     *
     * @param score - new ScoreInfo to add to our list of scores.
     */
    public void add(ScoreInfo score) {

        if (this.allScoresInfo.size() < this.tableSize) {
            this.allScoresInfo.add(score);
            this.sortingScores();
        } else {
            // no more place - check if higher than current scores.
            if (score.getScore() > this.allScoresInfo.get(this.allScoresInfo.size() - 1).getScore()) {
                this.allScoresInfo.remove(this.allScoresInfo.size() - 1);
                this.allScoresInfo.add(score);
                this.sortingScores();
            }
        }
    }

    /**
     * @return table size.
     */
    public int size() {
        return this.tableSize;
    }

    /**
     * @return the current high scores. The list is sorted such that the highest
     * scores come first.
     */

    public List<ScoreInfo> getHighScores() {
        // Already sorted.
        return this.allScoresInfo;
    }

    /**
     * return the rank of the current score: where will it be on the list if
     * added? Rank 1 means the score will be highest on the list. Rank `size`
     * means the score will be lowest. Rank > `size` means the score is too low
     * and will not be added to the list.
     *
     * @param score - score to check its rank.
     * @return the rank of the current score: where will it be on the list if
     * added(?).
     */

    public int getRank(int score) {
        for (int i = 1; i < this.allScoresInfo.size() + 1; i += 1) {
            int currentScore = this.allScoresInfo.get(i - 1).getScore();
            if (score > currentScore) {
                return i;
            }
        }
        return this.allScoresInfo.size() + 1; // will not be added to the list.
    }

    /**
     * Clears the table.
     */
    public void clear() {
        this.allScoresInfo.clear();
    }

    /**
     * Load table data from file. Current table data is cleared.
     *
     * @param filename - the specified file to save the data into him.
     * @throws IOException - exception that catch if there is a problem to write into
     *                     the file from any reason.
     */
    public void load(File filename) throws IOException {
        this.clear();
        FileInputStream fileInputStr = null;
        ObjectInputStream objInputStr = null;
        try {
            fileInputStr = new FileInputStream(filename);
            objInputStr = new ObjectInputStream(fileInputStr);
            // get the size of the table from the file.

            this.tableSize = objInputStr.readInt();

            for (int index = 0; index < this.tableSize; index += 1) {
                ScoreInfo s = (ScoreInfo) objInputStr.readObject();
                this.allScoresInfo.add(s);
            }
        } catch (Exception e) {
            System.out.println(" A problem to read the file ");
            e.printStackTrace();
        } finally { // finally - close the files
            try {
                fileInputStr.close();
                // if failed to close fileInputStr file, catch the exception
            } catch (IOException ex) {
                System.out.println(" A problem to close fileInputStr file (4) ");
                ex.printStackTrace();
            }

            try {
                objInputStr.close();
                // if failed to close objInputStr file, catch the exception
            } catch (IOException ex) {
                System.out.println(" A problem to close objInputStr file (5) ");
                ex.printStackTrace();
            }
        }

    }

    /**
     * Save table data to the specified file.
     *
     * @param filename - the specified file to save the data into him.
     * @throws IOException - exception that catch if there is a problem to write into
     *                     the file from any reason.
     */
    public void save(File filename) throws IOException {

        FileOutputStream fOutStream = null;
        ObjectOutputStream objOutStream = null;
        try {
            fOutStream = new FileOutputStream(filename);
            objOutStream = new ObjectOutputStream(fOutStream);
            objOutStream.writeInt(this.size()); // save the size of the table

            for (int index = 0; index < this.allScoresInfo.size(); index += 1) {
                ScoreInfo s = this.allScoresInfo.get(index);
                objOutStream.writeObject(s);
            }
        } catch (IOException io) {
            System.out.println("A problem to write the file (1) ");
            io.printStackTrace();
        } finally { // finally - close the files
            try {
                objOutStream.close();
                fOutStream.close();
                // if failed to close fOutStream file, catch the exception
            } catch (IOException ex) {
                System.out.println(" A problem to close fOutStream file (2) ");
                ex.printStackTrace();
            }
        }
    }

    /**
     * Read a table from file and return it.
     *
     * @param filename - the file that we will read a table from.
     * @return If the file does not exist, or there is a problem with reading
     * it, an empty table is returned.
     */

    public static HighScoresTable loadFromFile(File filename) {

        HighScoresTable emptyIfNeeded = new HighScoresTable(10);
        FileInputStream fileInputStr = null;
        ObjectInputStream objInputStr = null;
        HighScoresTable newHighScoreTable = new HighScoresTable(0);
        try {
            try {
                fileInputStr = new FileInputStream(filename);
                objInputStr = new ObjectInputStream(fileInputStr);
            } catch (IOException io) {
                return emptyIfNeeded;
            }
            // get the size of the table from the file.
            int tableSize = objInputStr.readInt();

            newHighScoreTable = new HighScoresTable(tableSize);

            for (int index = 0; index < tableSize; index += 1) {
                ScoreInfo s = (ScoreInfo) objInputStr.readObject();
                newHighScoreTable.allScoresInfo.add(s);
            }
            try {
                objInputStr.close();
            } catch (IOException i) {
                System.out.println(" can't close objInputStr file ");
            }
            return newHighScoreTable;
        } catch (Exception e) {
            if (!newHighScoreTable.allScoresInfo.isEmpty()) {
                return newHighScoreTable;
            }
            return emptyIfNeeded;
        }
    }

    /**
     * Sorting the list of scores.
     */
    public void sortingScores() {

        // algorithm sorting - Bubble sort.
        for (int i = 0; i < this.allScoresInfo.size(); i++) {
            for (int j = this.allScoresInfo.size() - 1; j > 0; j--) {
                if (this.allScoresInfo.get(j).getScore() > this.allScoresInfo.get(j - 1).getScore()) {
                    ScoreInfo tmp = this.allScoresInfo.get(j);
                    this.allScoresInfo.set(j, this.allScoresInfo.get(j - 1));
                    this.allScoresInfo.set(j - 1, tmp);
                }
            }
        }
    }
}
