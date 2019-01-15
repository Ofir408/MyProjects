import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

/**
 * 
 * @author Ofir Ben Shoham. This class write the result of the algorithm to the
 *         file that is called output.txt
 *
 */
public class SolutionsWriter {
	private String finalPath;
	private final String fileName = "output.txt";
	private int developedNodes;
	private int cost;

	/**
	 * @param finalPath
	 *            - the path of the solution.
	 * @param developedNodes
	 *            - developed Nodes of the solution.
	 * @param cost
	 *            - cost of the solution.
	 */
	public SolutionsWriter(String finalPath, int developedNodes, int cost) {
		super();
		this.finalPath = finalPath;
		this.developedNodes = developedNodes;
		this.cost = cost;
	}

	// write the result to the output.txt file.
	public void writeResult() {
		String strToWrite = finalPath + " " + String.valueOf(developedNodes) + " " + String.valueOf(cost);

		Writer writer;
		try {
			writer = new BufferedWriter(new FileWriter(fileName, false));
			System.out.println(strToWrite);
			writer.write(strToWrite);
			writer.close();
		} catch (IOException e) {
			System.out.println("Cant write to the file");
			e.printStackTrace();
		}
	}
}
