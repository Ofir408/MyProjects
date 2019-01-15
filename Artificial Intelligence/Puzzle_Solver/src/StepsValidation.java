import java.util.ArrayList;
import java.util.List;

/**
 * 
 * @author Ofir Ben Shoham. 
 * Given a state and a board number, returns the possible States to be played.
 *
 */
public class StepsValidation {

	// members
	private int boardSize;
	private StateOnBoard currentState;

	/**
	 * @param boardSize
	 * @param currentState
	 */
	public StepsValidation(int boardSize, StateOnBoard currentState) {
		this.boardSize = boardSize;
		this.currentState = currentState;
	}

	// makes the steps, returns a queue with the new states.
	// his size, represents the nodes that was created.

	// returns list with the possible steps from the current state.
	// For example - returns ["D", "U"] if Down & Up possible and Left & Right
	// aren't.
	public List<String> getPossibleListOptions() {
		String[][] stateArray = fromStringStateToArray();
		List<String> listToReturn = new ArrayList<>();
		listToReturn.add("U");
		listToReturn.add("D");
		listToReturn.add("L");
		listToReturn.add("R");

		int rowOfEmptyCell = -1, colOfEmptyCell = -1; // defaults.

		for (int r = 0; r < boardSize; r++)
			for (int c = 0; c < boardSize; c++) {
				if (stateArray[r][c].equals("0")) {
					rowOfEmptyCell = r;
					colOfEmptyCell = c;
				}
			}
		System.out.println("row of empty cell : " + rowOfEmptyCell + ", col is: " + colOfEmptyCell);
		if (rowOfEmptyCell == boardSize - 1) // up is not possible
			listToReturn.remove("U");

		if (rowOfEmptyCell == 0) // down is not possible
			listToReturn.remove("D");
		
		if (colOfEmptyCell == boardSize - 1)
			listToReturn.remove("L");

		if (colOfEmptyCell == 0) // right is not possible.
			listToReturn.remove("R");

		return listToReturn ;
	}

	// convert to double array (game board).
	private String[][] fromStringStateToArray() {
		String[] s = currentState.getCurrentBoardState().split("-");
		String[][] arrayToPrint = new String[boardSize][boardSize];
		int counter = 0;

		for (int r = 0; r < boardSize; r++)
			for (int c = 0; c < boardSize; c++) {
				arrayToPrint[r][c] = s[counter];
				counter += 1;
			}
		return arrayToPrint;
	}

}
