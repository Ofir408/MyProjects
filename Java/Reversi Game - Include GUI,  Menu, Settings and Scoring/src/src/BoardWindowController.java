
import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.util.ResourceBundle;
import java.util.Vector;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;
import javafx.stage.Stage;

/*
 * Created on: Jan 11, 2018
 *  Author Yuval Weinstein 
 */
public class BoardWindowController implements Initializable {
	
	@FXML
	private VBox root;
	
	@FXML
	private Label CurrentTurnLabel;
	
	@FXML
	private Label BlackScoreLabel;
	
	@FXML
	private Label WhiteScoreTable;
	
	@FXML
	private GridPane gameBoard;
	
	@FXML
	private Button closeButton;
	
	private Board board;
	

	@Override
	/**
	 * the function initializes the controller of boardWindow.
	 * it sets all the things that need to be ready before the game begins
	 * like the board's nodes, and then it starts the game.
	 */
	public void initialize(URL location, ResourceBundle resources) {
		
		File file = new File("gameData.txt");
		GameData.getDataFromText(file);
		this.board = new Board();
		final GameRunner gr = new GameRunner(this.board, 'X');
		this.setCurrentPlayerLabelByChar('X');
				
		int height = 400;
		int width = 400;
		
		int cellHeight = height / this.board.getLength();
		int cellWidth = width / this.board.getWidth();
		
		//inserting image views as well as rectangles to each compartment within the gridPane
		for (int i = 0; i < this.board.getLength(); i++) {
			for (int j = 0; j < this.board.getWidth(); j++) {
				Image image = this.getImageForSign(this.board.getCell(i, j).getSign());
				final ImageView im;
				im = new ImageView(image);
				im.setFitHeight(cellHeight);
				im.setFitWidth(cellWidth);
				Rectangle rect = new Rectangle(j*cellHeight, i*cellWidth ,cellHeight, cellWidth);
				rect.setFill(Color.LIGHTGREEN);
				rect.setStroke(Color.BLACK);
				final int row = j + 1;
				final int column = i + 1;
				
				//set functions regarding the new image view
				im.setOnMouseClicked((MouseEvent event) -> {
					char nextPlayer = gr.playNextMove(row, column);
					if (nextPlayer == ' ') {
						this.updateBoardGame();
						this.WhiteScoreTable.setText("second Player score : " + String.valueOf(this.board.getPlayerNumberOfPawns('O')));
						this.BlackScoreLabel.setText("first Player score : " + String.valueOf(this.board.getPlayerNumberOfPawns('X')));
						if (this.board.getPlayerNumberOfPawns('X') > this.board.getPlayerNumberOfPawns('O')) {
							this.CurrentTurnLabel.setText("first player has won!!!");
						} else if (this.board.getPlayerNumberOfPawns('O') > this.board.getPlayerNumberOfPawns('X')) {
							this.CurrentTurnLabel.setText("second player has won!!!");
						} else {
							this.CurrentTurnLabel.setText("it's a tie!!!");
						}
					} else {
						Vector<Cell> possibleCells = this.board.possibleCellsToAssign(nextPlayer);
						this.updateBoardGame();
						this.PaintPossibleCells(possibleCells);
						this.WhiteScoreTable.setText("second Player score : " + String.valueOf(this.board.getPlayerNumberOfPawns('O')));
						this.BlackScoreLabel.setText("first Player score : " + String.valueOf(this.board.getPlayerNumberOfPawns('X')));
						this.setCurrentPlayerLabelByChar(nextPlayer);
					}
					
				});
				
				im.setOnMousePressed((MouseEvent even) -> {
					
					for (Node node: this.gameBoard.getChildren()) {
						int x = GridPane.getRowIndex(node) + 1;
						int y = GridPane.getColumnIndex(node) + 1;
						if (x == row && y == column && node instanceof ImageView ) {
							ImageView imm = (ImageView)node;
							imm.setImage(new Image("PawnImages/yellowPressed.png"));
						}
					}
				});
				
				im.setOnMouseReleased((MouseEvent even) -> {
					for (Node node: this.gameBoard.getChildren()) {
						int x = GridPane.getRowIndex(node) + 1;
						int y = GridPane.getColumnIndex(node) + 1;
						if (x == row && y == column && node instanceof ImageView) {
							ImageView imm = (ImageView)node;
							imm.setImage(new Image("PawnImages/yellow.png"));
						}
					}
				});
								
				this.gameBoard.add(rect, i, j);
				this.gameBoard.add(im, i, j);
				this.PaintPossibleCells(this.board.possibleCellsToAssign('X'));
			}
		}
	}
	
	
	/**
	 * gets the image of the pawn by player.
	 * @param player is the sign of the player we want the image of
	 * @return the image of the pawn
	 */
	Image getImageForSign(char player) {
		try {
			if (player == 'X') {
				return this.mapNumberToImage(GameData.firstPlayer);
			} else if (player == 'O') {
				return this.mapNumberToImage(GameData.secondPlayer);
			} else {
				return new Image("PawnImages/noImage.jpg");
			}
		} catch (Exception e) {
			e.printStackTrace();
		} 
		return null;
	}
	
	/**
	 * the func maps a number to a pawn image. the mapping is specified in the settings menu.
	 * @param num is the number to map
	 * @return a pawn image by the number
	 */
	Image mapNumberToImage(int num) {
		switch(num) {
		case 1:
			return new Image("PawnImages/black.png");
		case 2:
			return new Image("PawnImages/white.png");
		case 3:
			return new Image("PawnImages/red.png");
		case 4:
			return new Image("PawnImages/pink.png");
		case 5:
			return new Image("PawnImages/green.png");
		default:
			return new Image("PawnImages/black.png");
		}
	}
	
	/**
	 * updated each node within gameBoard - changes the image $ rect by the board if needed.
	 */
	void updateBoardGame() {
		for (Node node: this.gameBoard.getChildren()) {
			ImageView imm;
			if(node instanceof ImageView) {
				imm = (ImageView)node;
				int i = GridPane.getRowIndex(node) + 1;
				int j = GridPane.getColumnIndex(node) + 1;
				Image imageToAssign = this.getImageForSign(this.board.getCell(i - 1, j - 1).getSign());
				imm.setImage(imageToAssign);
			}
		}
	}
	
	/**
	 * the func paints in yellow all possible cells to press and paints in white the ones who doesn't.
	 * @param possibleCells is a vector of all the possible cells in the board to press
	 */
	void PaintPossibleCells(Vector<Cell> possibleCells) {
		for (Node node: this.gameBoard.getChildren()) {
			ImageView imm;
			boolean found = false;
			if(node instanceof ImageView) {
				imm = (ImageView)node;
				int i = GridPane.getRowIndex(node) + 1;
				int j = GridPane.getColumnIndex(node) + 1;
				for (Cell cell: possibleCells) {
					if (i == cell.getX() && j == cell.getY()) {
						imm.setImage(new Image("PawnImages/yellow.png"));
						imm.setOnMousePressed((MouseEvent even) -> {
							imm.setImage(new Image("PawnImages/yellowPressed.png"));
						});
						found = true;
					}
				}
				if (!found) {
					imm.setOnMousePressed(null);
				}
			}
		}
	}
	
	/**
	 * sets the current label's string by the given player char/
	 * @param player is the player his turn arrived
	 */
	void setCurrentPlayerLabelByChar(char player) {
		if (player == 'X') {
			this.CurrentTurnLabel.setText("This is the turn of the first player");
		} else if (player == 'O') {
			this.CurrentTurnLabel.setText("This is the turn of the second player");
		}
	}
	
	@FXML
	/**
	 * Takes care to close the game and go back to the menu, when the user press of "back" button into the game.
	 * @throws IOException if doesn't find the new fxml (Menu.fxml).
	 */
	void closeButtonPressed() throws IOException {
		Stage stage = (Stage) closeButton.getScene().getWindow();
		AnchorPane root = (AnchorPane)FXMLLoader.load(getClass().getResource("Menu.fxml"));
		Scene scene = new Scene(root, 700, 600);
		stage.setTitle("Menu");
		stage.setScene(scene);
		stage.show();
	}
}
