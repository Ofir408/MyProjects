
/**
 * 
 * @author Ofir Ben Shoham.
 * The classes that will implements this interface will have abilities to save the path for the solution, cost of solution, and show the nodes number 
 * that was developed. 
 *
 */
public interface IGenerateStateDetails {
	
	// add a step to path, returns the new path.
	public String addStepToSolutionPath(StateOnBoard prevState, String step);
	
	// add cost to the current state of the solution.
	public void addCost (int costToAdd);
	
	// add nodes number that was developed in the last step.
	public void addDevelopedNodes(int currDevNodes);
	
}
