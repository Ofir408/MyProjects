
import java.io.IOException;
import java.net.URL;
import java.util.ResourceBundle;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.RadioButton;
import javafx.scene.control.ToggleGroup;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

/*
 * Created on: Jan 06, 2018
 *  Author Ofir Ben-Shoham.
 */
public class MenuControler implements Initializable {
	@FXML
	private RadioButton startButton;
	@FXML
	private RadioButton settingsButton;
	@FXML
	private Node enterButton;
	@FXML
	private Button exit;
	private ToggleGroup options;

	@Override
	/**
	 * set the toggle group of the possible options.
	 */
	public void initialize(URL location, ResourceBundle resources) {
		options = new ToggleGroup();
		this.settingsButton.setToggleGroup(options);
		this.startButton.setToggleGroup(options);
	}

	@FXML
	/**
	 * Call to this function if enter was pressed by the user from the menu.
	 * @throws IOException if thhe exml not found.
	 */
	public void enterWasPressed() throws IOException {
		// check which option the user chose as follows.
		if (this.options.getSelectedToggle().equals(startButton)) {
			// the player chose to start in the game

			VBox root = (VBox) FXMLLoader.load(getClass().getResource("BoardWindow.fxml"));
			Stage currentStage = (Stage) enterButton.getScene().getWindow();
			currentStage.close(); // close the current scene.
			Scene gameScene = new Scene(root, 700, 600);
			currentStage.setTitle("Game Screen");
			currentStage.setScene(gameScene);
			currentStage.show();

		} else {
			// the player chose to see the settings screen.
			/**
			 * Show the settings screen.
			 */
			// open new scene with the settings screen as needed
			AnchorPane root = (AnchorPane) FXMLLoader.load(getClass().getResource("Settings.fxml"));
			Stage currentStage = (Stage) enterButton.getScene().getWindow();
			currentStage.close(); // close the current scene.
			Scene settingsScene = new Scene(root, 700, 600); // define new scene
																// for the
																// settings
																// screen.
			currentStage.setTitle("Settings Screen");
			currentStage.setScene(settingsScene);
			currentStage.show();
		}
	}

	@FXML
	/**
	 * This function is called when the user pressed "exit" from the menu.
	 */
	public void exitFromMenu() {
		System.out.println("The user wanted to exit from the menu");
		System.exit(0);
	}

}
