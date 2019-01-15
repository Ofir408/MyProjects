import java.util.Queue;

/**
 * 
 * @author Ofir Ben Shoham. 
 * IDS algorithm implementation.
 *
 */
public class IDS extends AbstractAlgorithm {

	
	public IDS(GameInitialDetails gameDetails) {
		super(gameDetails);
	}

	// returns the children number of the son the this node in that path
	// It equivalent to return the number of the brothers that this node in this
	// path has.
	/*private int getChildrenNum(String sonPath) {
		String fatherPath = sonPath.substring(0, sonPath.length() - 1);
		StateOnBoard fatherState = getState(fatherPath);

		System.out.println("path of father is:" + fatherState.getPath() + ", but it should be: " + fatherPath);
		BoardPrinter.getInstance().printState(fatherState.getCurrentBoardState(), 3);
		StateOnBoard backup = super.currentBoardState.clone();
		super.currentBoardState = fatherState;
		int num = playOneStep().size();
		super.currentBoardState = backup; // restore.
		return num;
	}*/

	// get state object of this path.
	/*private StateOnBoard getState(String path) {
		StateOnBoard currState = new StateOnBoard(super.gameInitialDetails.getInitialBoardState(), "", 0, 0);
		StateOnBoard temp = super.currentBoardState.clone(); // backup.
		super.currentBoardState = currState;

		char[] sides = path.toCharArray();
		for (char c : sides) {
			String currStep = c + ""; // convert to string
			currState = makeStepHelper(currStep);
			super.currentBoardState = currState; // restore.
		}
		super.currentBoardState = temp; // restore.
		return currState;
	}*/

	// Documentation in base class.
	@Override
	public StateOnBoard findSolution() {
		StateOnBoard backup = super.currentBoardState;
		int depth = 0;
		do {
			StateOnBoard result = DLS(super.currentBoardState, 0, depth);
			if (result != null) {
				// found
				

				System.out.println("Depth is: " + depth);
				result.addCost(depth);
				result.addDevelopeNodes(-result.getDevelopedNodesNum() + super.developedNode + 1); // +1 since we developed the first node.

				System.out.println("childern num is: " + result.getDevelopedNodesNum());
				System.out.println("depth is: " + result.getCost());
				return result;
			}
			super.currentBoardState = backup;
			// reset for the next iteration, since we were asked to show the developed nodes in the *last* iteration when implementing IDS.
			super.developedNode = 0; 
		} while (++depth < 1000);
		return null; // not found
	}

	private StateOnBoard DLS(StateOnBoard state, int depth, int realDepth) {
		super.currentBoardState = state;
		System.out.println("CurrDepth is: " + depth);
		if (realDepth == depth) {
			if (state.getCurrentBoardState().equals(super.goalBoardState))
				return state;
			else
				System.out.println(
						"curr state is: " + state.getCurrentBoardState() + ", goal is: " + super.goalBoardState);
			return null; // not found.
		} else if (depth < realDepth) {
			super.currentBoardState = state;
			Queue<StateOnBoard> childerns = playOneStep();
			int newNodesLength = childerns.size();
			for (int i = 0; i < newNodesLength; i++) {
				StateOnBoard s = childerns.remove();
				super.addDevelopedNodes(1);
				System.out.println("------------------- path : " + s.getPath());
				StateOnBoard result = DLS(s, depth + 1, realDepth);
				if (result != null)
					return result;
			}
		}
		return null; // not found
	}

	/*
	 * @Override public StateOnBoard findSolution() { // IDS implementation
	 * StateOnBoard solutionState = null;
	 * 
	 * while (solutionState == null) { depth++; System.out.println("depth is: "
	 * + depth); if (depth > 50) break; solutionState = dfs(); } return
	 * solutionState; }
	 */

	/*
	 * private boolean checkIfLowerTheMax(Stack<StateOnBoard> stack, int max) {
	 * Stack<StateOnBoard> newStack = new Stack<StateOnBoard>();
	 * newStack.addAll(stack);
	 * 
	 * for (StateOnBoard s : newStack) if (s.getPath().length() < max) {
	 * System.out.println("Path is: " + s.getPath() + ", max is: " + max);
	 * System.out.println("true returned"); return true; }
	 * System.out.println("false returned"); return false; }
	 */

	/*
	 * private StateOnBoard dfs() { Stack<StateOnBoard> stack = new
	 * Stack<StateOnBoard>(); stack.push(super.currentBoardState);
	 * 
	 * while (!stack.isEmpty()) { StateOnBoard currState = stack.pop(); if
	 * (currState.getCurrentBoardState().equals(super.goalBoardState)) { //
	 * found the solution. return currState; }
	 * System.out.println("depth of current cell is: " +
	 * currState.getPath().length()); /*if (currState.getPath().length() >=
	 * depth) return null; // solution not found in this depth.
	 */

	/*
	 * super.currentBoardState = currState; Queue<StateOnBoard> successors =
	 * super.playOneStep(); int length = successors.size(); for (StateOnBoard s
	 * : successors) { s.addDevelopeNodes(length); s.addCost(1); stack.push(s);
	 * } if(!checkIfLowerTheMax(stack, depth)) return null; // not found in that
	 * length; } return null; // not found in his depth. }
	 */

	@Override
	// the depth of this algorithm.
	public void addCost(int newCost) {
		this.currentBoardState.addCost(newCost);
	}
}
