import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Stage;

/**
 * 
 * Created on: Jan 7, 2018 Names : Yuval Weinstein & Ofir Ben Shoham. Id:
 * 208580613 & 208642496.
 */

public class Main extends Application {
	
	@Override
	/**
	 * Start from the menu screen, and show it.
	 */
	public void start(Stage primaryStage) {
		try {
			AnchorPane root = (AnchorPane) FXMLLoader.load(getClass().getResource("Menu.fxml"));
			Scene scene = new Scene(root, 600, 600);
			primaryStage.setTitle("Menu");
			primaryStage.setScene(scene);
			primaryStage.show();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Run the game
	 * @param args - nothing in our case, since there is no main arguments.
	 */
	public static void main(String[] args) {
		try {
			launch(args);
		} catch (Exception e) {
			System.out.println("The game is ended");
		}
	}

}
