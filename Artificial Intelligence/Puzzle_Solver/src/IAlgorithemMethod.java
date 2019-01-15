import java.util.Queue;

/**
 * 
 * @author Ofir Ben Shoham
 *
 */
public interface IAlgorithemMethod {
	// makes one step according the algorithm.
	// returns the new board state.
	public Queue<StateOnBoard>  playOneStep();  
	
	// Run the  and returns the solution (if there is).
	// If no solution was found - return -1.
	public StateOnBoard findSolution(); 
	
	// returns the number of the algorithm according to his type.
	public int algorithemNumber(); 
	
	// get the goal state.
	public String goalState(); 

}
