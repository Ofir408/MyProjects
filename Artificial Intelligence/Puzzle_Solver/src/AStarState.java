
public class AStarState extends StateOnBoard{

	private double gFuncVal; // value, using g function (cost of the path).
	private double hFunc; // value using h (my heuristic function).
	private int creationTime;
	// we need to remember that f(x) = g(x) + h(x).
	
	
	public AStarState(String currentBoardState, String path, int developedNodesNum, int cost, int creationTime) {
		super(currentBoardState, path, developedNodesNum, cost);
		this.creationTime = creationTime; 
	}


	// getters & setters functions.
	public double getgFuncVal() {
		return gFuncVal;
	}
	
	public int getCreationTime() {
		return this.creationTime; 
	}

	public void setgFuncVal(double gFuncVal) {
		this.gFuncVal = gFuncVal;
	}

	public double gethFunc() {
		return hFunc;
	}

	public void sethFunc(double hFunc) {
		this.hFunc = hFunc;
	}
}
