/**\
 * 
 * @author Ofir Ben Shoham.
 * Factory Design Pattern, returns the algorithm after getting his number.
 *
 */
public class AlgorithmFactory {

	// return AbstractAlgorithm according the input algorithemNum.
	public static AbstractAlgorithm generateAlgorithem(GameInitialDetails gameDetails, int algorithemNum) {
		switch (algorithemNum) {
		case 1:
			return new IDS(gameDetails);
		case 2:
			return new BFS(gameDetails);
		default:
			return new AStar(gameDetails);
		}
	}
	
}
