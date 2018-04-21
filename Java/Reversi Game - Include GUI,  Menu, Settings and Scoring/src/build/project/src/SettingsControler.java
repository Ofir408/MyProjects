
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.net.URL;
import java.util.ResourceBundle;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.TextField;
import javafx.scene.layout.AnchorPane;
import javafx.scene.paint.Color;
import javafx.scene.text.Text;
import javafx.stage.Stage;

/**
 * Created on: Jan 06, 2018
 *  Author Ofir Ben-Shoham.
 */

public class SettingsControler implements Initializable {

	@FXML
	private TextField boardSize;
	@FXML
	private TextField firstPlayerColor;
	@FXML
	private TextField secondPlayerColor;
	@FXML
	private Node saveSettingsButton;
	@FXML
	private Button closeButton;
	@FXML
	private Text one;
	@FXML
	private Text two;
	@FXML
	private Text size;


	
	/**
	 * This function is called when the save settings button is pressed. We want
	 * to save the new settings to gameData file.
	 * @throws IOException 
	 */
	@FXML
	public void saveSettingsButtonPressed() throws IOException {
		// first get the data that the user entered.
		String firstPlayerColorNum, secondPlayerColorNum, sizeOfBoard;
		int firstPlayerColorNumInt, secondPlayerColorNumInt, sizeOfBoardInt;


		sizeOfBoard = boardSize.getText();
		firstPlayerColorNum = firstPlayerColor.getText();
		secondPlayerColorNum = secondPlayerColor.getText();

		// convert them to integer.

		try {
			firstPlayerColorNumInt = Integer.parseInt(firstPlayerColorNum);
		} catch (Exception e) {
			firstPlayerColorNumInt = 0;
		}
		
		try {
			secondPlayerColorNumInt = Integer.parseInt(secondPlayerColorNum);
		} catch (Exception e) {
			secondPlayerColorNumInt = 0;
		}
		
		try {
			sizeOfBoardInt = Integer.parseInt(sizeOfBoard);
		} catch (Exception e) {
			sizeOfBoardInt = 8;
		}
		

		// check if there are legal inputs
		if (firstPlayerColorNumInt < 0 || firstPlayerColorNumInt > 5) {
			// set the default (1)
			firstPlayerColorNumInt = 1; // default value.
		}
		// the same for the second color of the player.
		if (secondPlayerColorNumInt < 0 || secondPlayerColorNumInt > 5) {
			// set the default (1)
			secondPlayerColorNumInt = 1; // default value.
		}

		// check the board's size.
		if (sizeOfBoardInt < 4 || sizeOfBoardInt > 20) {
			sizeOfBoardInt = 8; // default value.
		}

		// then, return to the menu screen.
		AnchorPane root = (AnchorPane) FXMLLoader.load(getClass().getResource("Menu.fxml"));
		Stage currentStage = (Stage) saveSettingsButton.getScene().getWindow();
		currentStage.close(); // close the current scene.
		Scene settingsScene = new Scene(root, 600, 600); // define new scene
															// for the
															// settings
															// screen.
		currentStage.setTitle("Menu Screen");
		currentStage.setScene(settingsScene);
		currentStage.show();
		// then call to helper function that writes the data to gameData file.
		writingNewDataToFile(firstPlayerColorNumInt, secondPlayerColorNumInt, sizeOfBoardInt);
	}
	
	/**
	 * Getting firstPlayerColorNumInt, secondPlayerColorNumInt, sizeOfBoardInt - first color, second color & size of board.
	 * That the user chose, then writing them to the file.
	 */
	private void writingNewDataToFile(int firstPlayerColorNumInt, int secondPlayerColorNumInt,
			int sizeOfBoardInt) {
		String pathOfFile = "gameData.txt";
		try {
			BufferedWriter writerDescriptor = new BufferedWriter(new FileWriter(pathOfFile));
			/*
			 * First player is: 2. First player color is: Black. Second player
			 * color is: White. Size of the board: 9.
			 */
			PrintWriter writer = new PrintWriter(pathOfFile);
			writer.print("");
			writer.close();

			writerDescriptor.write("First player color is: " + firstPlayerColorNumInt + ".\n");
			writerDescriptor.write("Second player color is: " + secondPlayerColorNumInt + ".\n");
			writerDescriptor.write("Size of the board: " + sizeOfBoardInt + ".");
			writerDescriptor.close();
		} catch (IOException e) {
			System.out.println("Can't write to the file, into writingNewDataToFile ");
		}

	}


@Override
	/**
	 * initialize method that show the current settings.
	 */
	public void initialize(URL location, ResourceBundle resources) {
		GameData.getDataFromText(new File("gameData.txt"));
		
		this.one.setText(String.valueOf(GameData.firstPlayer));
		this.one.setFill(Color.GREEN);
		this.two.setText(String.valueOf(GameData.secondPlayer));
		this.two.setFill(Color.RED);
		this.size.setText(String.valueOf(GameData.RowNumber));
		this.size.setFill(Color.GREEN);
	}
	
	@FXML
	/**
	 *  This function takes care to go back to the menu from the settings screen.
	 * @throws IOException - if the file doesnt found.
	 */
	public void goBackWasPressed() throws IOException {
	    Stage stage = (Stage) closeButton.getScene().getWindow();
	    //stage.close();
		AnchorPane root = (AnchorPane) FXMLLoader.load(getClass().getResource("Menu.fxml"));
		Scene scene = new Scene(root, 600, 600);
		stage.setTitle("Menu");
		stage.setScene(scene);
		stage.show();
	}

}
