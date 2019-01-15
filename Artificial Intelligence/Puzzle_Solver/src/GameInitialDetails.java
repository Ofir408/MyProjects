/**
 * 
 * @author Ofir Ben Shoham
 * This class represents the initial details that the game has. 
 * Takes the details from the input text file, and returns this class.
 */
public class GameInitialDetails {
	
	private int algorithemNumber;
	private int boardSize;
	private String initialBoardState; 

	
	public GameInitialDetails(int algorithemNumber, int boardSize, String initialBoardState) {
		this.algorithemNumber = algorithemNumber;
		this.boardSize = boardSize;
		this.initialBoardState = initialBoardState;
	}
	
	
	
	// getters.
	public int getAlgorithemNumber() {
		return algorithemNumber;
	}
	public int getBoardSize() {
		return boardSize;
	}
	public String getInitialBoardState() {
		return initialBoardState;
	}

}
