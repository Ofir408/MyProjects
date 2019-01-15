
public class BoardPrinter {
	private BoardPrinter() {}
	private static BoardPrinter boardPrinter = new BoardPrinter();
	
	public static BoardPrinter getInstance() {
		return boardPrinter; 
	}
	
	public void printState(String stateStringToPrint, int boardSize) {

		 String[][] boardStateArray = fromStringStateToArray(stateStringToPrint, boardSize);
		 printTheBoard(boardStateArray);
	}
	
	private String[][] fromStringStateToArray(String currentBoardState, int boardSize) {
		String[] s = currentBoardState.split("-");
		String[][] arrayToPrint = new String [boardSize][boardSize]; 
		int counter = 0; 
		
		for (int r = 0; r < boardSize; r ++)
			for (int c = 0; c < boardSize; c++) {
				arrayToPrint[r][c] = s[counter];
				counter += 1; 
			}
		return arrayToPrint; 
	}
	
	
	private static void printTheBoard(String[][] arrayToPrint) {

		/** Print the game board */
		for (int row = 0; row < arrayToPrint.length; ++row) {
			for (int col = 0; col < arrayToPrint[0].length; ++col) {
				printCell(arrayToPrint[row][col]); // print each of the cells
				if (col != arrayToPrint[0].length - 1) {
					System.out.print("|"); // print vertical partition
				}
			}
			System.out.println();
			if (row != arrayToPrint.length - 1) {
				System.out.println("-----------"); // print horizontal partition
			}
		}
		System.out.println();
	}

	/** Print a cell with the specified "content" */
	private static void printCell(String content) {
		if (content.equals("0"))
			System.out.print("   ");
		else {
				System.out.print(content + "  ");
		}

	}

}
