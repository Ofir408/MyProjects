import java.util.LinkedList;
import java.util.Queue;

/**
 * 
 * @author Ofir Ben Shoham. 
 * BFS algorithm implementation.
 */
public class BFS extends AbstractAlgorithm {

	public BFS(GameInitialDetails gameDetails) {
		super(gameDetails);
	}

	// Documentation of these functions in base class.

	@Override
	public void addCost(int currCost) {
		// cost in 0 always in BFS according the definitions of this exercise.
	}

	@Override
	public StateOnBoard findSolution() {

		// BFS Implementation, as requested.
		Queue<StateOnBoard> L = new LinkedList<StateOnBoard>();
		L.add(super.currentBoardState); // initial node.

		while (!L.isEmpty()) {
			StateOnBoard node = L.remove();
			super.addDevelopedNodes(1); // since here we developed this node.

			// check if we reached the goal state
			System.out.println("goal is: " + goalBoardState);
			System.out.println("curr is: " + node.getCurrentBoardState());
			if (node.getCurrentBoardState().equals(goalBoardState)) {
				node.addDevelopeNodes(super.getDevelopeNodes() - node.getDevelopedNodesNum());
				return node; // found.
			}

			super.currentBoardState = node;
			Queue<StateOnBoard> newCreatedNodes = playOneStep();

			System.out.println("size of created node queue is: " + newCreatedNodes.size());
			L.addAll(newCreatedNodes);
		}
		System.out.println("No solution, null was returned");
		return null; // no solution.
	}
}
