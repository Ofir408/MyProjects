import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
/**
 * 
 * @author Ofir Ben Shoham..
 * Abstract class that represents base algorithm class.
 * Each Algorithm (BFS/IDS/A star) extends this class.
 *
 */
public abstract class AbstractAlgorithm implements IAlgorithemMethod, IGenerateStateDetails {

	/**
	 * add step to the path of this state.
	 */
	@Override
	public String addStepToSolutionPath(StateOnBoard prevState, String step) {
		return prevState.getPath() + step;
	}

	// members, protected in order to user them in derived classes.
	protected GameInitialDetails gameInitialDetails;
	protected StateOnBoard currentBoardState;
	protected String goalBoardState;
	protected int developedNode = 0;

	// got gameDetails  - the object from the data in "input" text file.
	public AbstractAlgorithm(GameInitialDetails gameDetails) {
		this.gameInitialDetails = gameDetails;
		this.currentBoardState = new StateOnBoard(gameDetails.getInitialBoardState(), "", 0, 0);
		this.goalBoardState = goalState(); // O(1) access to the goal state in derived classes.
	}

	@Override
	// get the goal state (the solution) - as a string.
	public String goalState() {
		int boardSize = this.gameInitialDetails.getBoardSize();
		String goalState = "";
		int numbersInBoard = boardSize * boardSize - 1;

		for (int index = 1; index <= numbersInBoard; index++) {
			goalState += String.valueOf(index) + "-";
		}

		return goalState + "0"; // "0" - represents empty cell on the board.
	}

	// returns queue of states on board, does just the legal steps.
	public Queue<StateOnBoard> playOneStep() {
		System.out.println("new path : " + this.currentBoardState.getPath());

		Queue<StateOnBoard> createdNodes = new LinkedList<StateOnBoard>();
		StepsValidation stepsValidation = new StepsValidation(gameInitialDetails.getBoardSize(), currentBoardState);
		List<String> validSteps = stepsValidation.getPossibleListOptions();
		System.out.println("validSteps length: " + validSteps.size());

		for (String currValidStep : validSteps) {
			StateOnBoard currCreatedState = makeStepHelper(currValidStep);
			createdNodes.add(currCreatedState);
		}

		return createdNodes;
	}

	// helper method that got a string that represents a side, and returns the state after making this step 
	// on the current board state.
	protected StateOnBoard makeStepHelper(String side) {
		StepMaker stepMaker = new StepMaker(gameInitialDetails.getBoardSize(), currentBoardState);
		String newPath;
		String newBoardState = "";

		switch (side) {

		case "U":
			newBoardState = stepMaker.upStep();
			break;
		case "D":
			newBoardState = stepMaker.downStep();
			break;

		case "L":
			newBoardState = stepMaker.leftStep();
			break;
		case "R":
			newBoardState = stepMaker.rightStep();
			break;
		}

		newPath = addStepToSolutionPath(currentBoardState, side);
		System.out.println("new path is: " + newPath);
		return new StateOnBoard(newBoardState, newPath, currentBoardState.getDevelopedNodesNum(),
				currentBoardState.getCost());
	}

	// abstract class, will be implemented in derived classes.
	@Override
	public abstract StateOnBoard findSolution();

	@Override
	public int algorithemNumber() {
		return this.gameInitialDetails.getAlgorithemNumber();
	}

	// returns false if we already reach the goal state, otherwise true.
	/*
	 * private boolean canContinue() { return
	 * this.currentBoardState.equals(this.goalBoardState); }
	 */

	/*
	 * (non-Javadoc)
	 * 
	 * @see IGenerateStateDetails#addCost(int)
	 */
	@Override
	public abstract void addCost(int currCost);

	/*
	 * (non-Javadoc)
	 * 
	 * @see IGenerateStateDetails#addDevelopedNodes(int)
	 */
	@Override
	public void addDevelopedNodes(int newDevNodes) {
		this.currentBoardState.addDevelopeNodes(newDevNodes);
		this.developedNode += newDevNodes;
		System.out.println("developedNode is: " + this.currentBoardState.getDevelopedNodesNum());
	}

	public int getDevelopeNodes() {
		return this.developedNode;
	}
}
