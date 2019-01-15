
/**
 * 
 * @author Ofir Ben Shoham. Gets the current state and the board size. Makes a
 *         step (U/D/L/R) and returns the new state.
 *
 */
public class StepMaker {

	private int boardSize;
	private StateOnBoard currentState;

	
	public StepMaker(int boardSize, StateOnBoard currentState) {
		this.boardSize = boardSize;
		this.currentState = currentState;
	}

	/*
	 * private Pair<Integer, Integer> findEmptyCell() {
	 * 
	 * String[] s = currentState.getCurrentBoardState().split("-"); String[][]
	 * boardArray = new String[boardSize][boardSize]; int counter = 0;
	 * 
	 * for (int r = 0; r < boardSize; r++) for (int c = 0; c < boardSize; c++) {
	 * boardArray[r][c] = s[counter]; counter += 1; }
	 * 
	 * for (int r = 0; r < boardSize; r++) for (int c = 0; c < boardSize; c++) {
	 * if (boardArray[r][c].equals("0")) { return new Pair<Integer, Integer>(r,
	 * c); } }
	 * System.out.println("Into StepMaker:findEmptyCell, empty cell not found");
	 * return new Pair<Integer, Integer>(-1, -1); }
	 */

	/*private int occOfCharAtStr(String str, char c) {
		int counter = 0;
		char[] charArray = str.toCharArray();
		for (char currChar : charArray)
			if (currChar == c)
				counter++;
		return counter;
	}*/
	
	private String replaceStr(int secInxDistanceFromEmpty) {
	    String[] numbers = this.currentState.getCurrentBoardState().split("-");
	    int length = numbers.length; 
	    int emptyCellInx = 0; 
	    for (int i=0; i<length; i++)
	    	if (numbers[i].equals("0"))
	    		emptyCellInx = i ;
	    
	    int secInx = emptyCellInx + secInxDistanceFromEmpty;
	    
	    if (secInx < 0)
	    	System.out.println("secInx <0 , " + secInx);
	    
	    // swap
	    String temp = numbers[secInx];
	    numbers[secInx] = numbers[emptyCellInx];
	    numbers[emptyCellInx] = temp; 
	    
	    return backToString(numbers);
	}
	
	private String backToString(String [] arr) {
		String toReturn = "";
		int lengthWithoutLast = arr.length - 1;

		for (int i = 0; i < lengthWithoutLast; i++) {
			toReturn += arr[i] + "-";
		}
		toReturn += arr[lengthWithoutLast];
		return toReturn; 
	}

	// replace between char a and char b into the string str
	// returns the new string
	/*private String getReplacedString(int secondInx) {

		String currentBoardStr = this.currentState.getCurrentBoardState();
		String toReturn = "";

		// StringBuilder currentBoardStrBuilder = new
		// StringBuilder(currentBoardStr);

		if (currentBoardStr.contains("-0")) {
			int zeroIndex = currentBoardStr.indexOf("0");
			int count = occOfCharAtStr(currentBoardStr.substring(0, zeroIndex), '-');
			System.out.println("currentBoardStr is: " + currentBoardStr + ", secInx is: " + secondInx);
			int secCharCounter = occOfCharAtStr(currentBoardStr.substring(0, secondInx), '-');
			int realIndex = zeroIndex - count;

			String[] stateBoardArray = this.currentState.getCurrentBoardState().split("-");

			// swap
			String tempValue = stateBoardArray[realIndex];
			stateBoardArray[realIndex] = stateBoardArray[secCharCounter];
			stateBoardArray[secCharCounter] = tempValue;

			int lengthWithoutLast = stateBoardArray.length - 1;

			for (int i = 0; i < lengthWithoutLast; i++) {
				toReturn += stateBoardArray[i] + "-";
			}
			toReturn += stateBoardArray[lengthWithoutLast];

			/*
			 * System.out.println(
			 * "---------------------------------------------------");
			 * BoardPrinter.getInstance().printState(toReturn, boardSize);
			 * System.out.println(
			 * "---------------------------------------------------");
			 */

			//return toReturn;

			/*
			 * char charToReplace = currentBoardStr.charAt(secondInx);
			 * 
			 * 
			 * currentBoardStrBuilder.setCharAt(index, charToReplace);
			 * currentBoardStrBuilder.setCharAt(secondInx, '0'); // put the
			 * empty // cell there. ; String strToReturn =
			 * currentBoardStrBuilder.toString(); System.out.println(
			 * "---------------------------------------------------");
			 * BoardPrinter.getInstance().printState(strToReturn, boardSize);
			 * System.out.println(
			 * "---------------------------------------------------");
			 * 
			 * return currentBoardStrBuilder.toString();
			 *
		}
		System.out.println("Right step  - no ready was made");
		return this.currentState.getCurrentBoardState();
	}*/

	public String rightStep() {

		/*String currentBoardStr = this.currentState.getCurrentBoardState();

		if (currentBoardStr.contains("-0")) {
			int index = currentBoardStr.indexOf("-0") + 1;
			int secCharInx = index - 2;

			return getReplacedString(secCharInx);
		}
		return currentBoardStr;*/
		return replaceStr(-1); // -1 to replace right.
	}

	public String leftStep() {
		/*String currentBoardStr = this.currentState.getCurrentBoardState();

		if (currentBoardStr.contains("-0")) {
			int index = currentBoardStr.indexOf("-0") + 1;
			int secCharInx = index + 2;

			return getReplacedString(secCharInx);
		}
		return currentBoardStr;*/
		
		return replaceStr(1); // +1 to replace left.
	}

	public String upStep() {
		/*String currentBoardStr = this.currentState.getCurrentBoardState();

		if (currentBoardStr.contains("-0")) {
			int index = currentBoardStr.indexOf("-0") + 1;
			int secCharInx = index + 2 * boardSize;

			return getReplacedString(secCharInx);
		}
		return currentBoardStr;*/
		return replaceStr(boardSize);
	}

	public String downStep() {
		/*String currentBoardStr = this.currentState.getCurrentBoardState();

		if (currentBoardStr.contains("-0")) {
			int index = currentBoardStr.indexOf("-0") + 1;
			int secCharInx = index - 2 * boardSize;

			if (secCharInx < 0) {
				System.out.println("index is: " + String.valueOf(index) +  ", secCharInx is: " + String.valueOf(secCharInx));
				System.out.println("StepMaker : downStep() ,  Problem in the index, it's : " + secCharInx);
				System.out.println("(problem) when the board is: " + currentBoardStr);
				BoardPrinter.getInstance().printState(currentBoardStr, boardSize);
			}

			return getReplacedString(secCharInx);
		}
		return currentBoardStr;*/
		return  replaceStr(-boardSize);

	}
}
