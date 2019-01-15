
/**
 * 
 * @author Ofir Ben Shoham.
 * This class represents a state into the board.
 */
public class StateOnBoard {
	
	private String currentBoardState; 
	private String path; 
	private int developedNodesNum; 
	private int cost; 


	
	/**
	 * @param currentBoardState - string "1-2-3 ... " , that represents the current places of the number on the board.
	 * @param path - the path until the current state.
	 * @param developedNodesNum - int.
	 * @param cost of the solution
	 */
	public StateOnBoard(String currentBoardState, String path, int developedNodesNum, int cost) {
		this.currentBoardState = currentBoardState;
		this.path = path;
		this.developedNodesNum = developedNodesNum;
		this.cost = cost;
	}
	
	/**
	 * @return the currentBoardState
	 */
	public String getCurrentBoardState() {
		return currentBoardState;
	}
	/**
	 * @return the path
	 */
	public String getPath() {
		return path;
	}
	/**
	 * @return the developedNodesNum
	 */
	public int getDevelopedNodesNum() {
		return developedNodesNum;
	}
	/**
	 * @return the cost
	 */
	public int getCost() {
		return cost;
	}

	public void addDevelopeNodes(int newDevNodes) {
		this.developedNodesNum += newDevNodes;
	}
	
	public void addCost(int newCost) {
		this.cost += newCost; 
	}
	
	public StateOnBoard clone() {
		return new StateOnBoard(this.currentBoardState, this.path, this.developedNodesNum, this.cost);
	}

}
