import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

/**
 * 
 * @author Ofir Ben Shoham
 * This class takes care to read the data from the input text file.
 */
public class InputTextReader {
	
	private String fileName; // the name of input file.
	
	// constructor.
	public InputTextReader(String name) {
		this.fileName = name; 
	}
	
	// read from the input file and returns GameInitialDetails class that has the initial details.
	public GameInitialDetails getInitialGameDetails() {
		int algorithemNumber = 0;
		int boardSize = 0;
		String initialBoardState = null; 
		

		try (BufferedReader br = new BufferedReader(new FileReader(this.fileName))) {

		    algorithemNumber = Integer.parseInt(br.readLine()); // Algorithm  number.
		    boardSize = Integer.parseInt(br.readLine());
		    initialBoardState = br.readLine();
		    } catch (NumberFormatException n) {
		    	System.out.println("Error while reading into InputTextReader: getInitialGameDetails()");
		    } catch (FileNotFoundException f) {
		    	if (!fileName.contains("src")) {
		    		this.fileName = "src/" + this.fileName;
	    			return getInitialGameDetails(); // check if the user put the file under src library.
		    	}
		    	System.out.println("File not found exception into InputTextReader: getInitialGameDetails()");
		    } catch (IOException i ) {
		    	System.out.println("IOException  into InputTextReader: getInitialGameDetails()");
		}
		return new GameInitialDetails(algorithemNumber, boardSize, initialBoardState);
	}
}
