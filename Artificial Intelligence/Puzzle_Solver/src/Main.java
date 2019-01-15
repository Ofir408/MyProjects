/**
 * 
 * @author Ofir Ben Shoham.
 * Main function that executes the code.
 *
 */
public class Main{

	public static void main(String[] args) {
		// read input from text file
		String fileName = "input.txt"; // input file name.
		InputTextReader inputTextReader = new InputTextReader(fileName);
		GameInitialDetails gameInitialDetails = inputTextReader.getInitialGameDetails();
		int boardSize = gameInitialDetails.getBoardSize();
		String stateToPrint = gameInitialDetails.getInitialBoardState();
		System.out.println("stateToPrint is: " + stateToPrint + ", board size is: " + String.valueOf(boardSize));
		BoardPrinter.getInstance().printState(stateToPrint, boardSize);

		// Using Factory Design Pattern.
		AbstractAlgorithm abstractAlgo = AlgorithmFactory.generateAlgorithem(gameInitialDetails,
				gameInitialDetails.getAlgorithemNumber());
		
		StateOnBoard solutionState = abstractAlgo.findSolution();
		System.out.println("--------------- Solution was found! ------------------");
		System.out.println("path of solution is: " + solutionState.getPath());
		System.out.println("developed nodes number is: " + solutionState.getDevelopedNodesNum());
		System.out.println("cost: " + solutionState.getCost());
		System.out.println("--------------- ----------- ------------------");
		
		// write the solution to the output file.
		new SolutionsWriter(solutionState.getPath(), solutionState.getDevelopedNodesNum(), solutionState.getCost()).writeResult();
	}
}
