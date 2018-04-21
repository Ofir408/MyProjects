import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

/*
/*
 * Created on: Jan 06, 2018
 *  Author Ofir Ben Shoham. 
 */

public class GameData {
	public static int RowNumber = 8; //number of rows in the board
	public static int ColNumber = 8; //number of columns in the board
	public static int firstPlayer, secondPlayer; //colors of pawns of the players
	public static int playerThatStartInGame;

	/**
	 * static method that updates the data of the game (such as players colors & size of board), from the settings file.
	 * @param fileDes - the settings file to get the data from.
	 */
	static void getDataFromText(File fileDes) {
		BufferedReader reader = null;
		int firstPlayerIndex, firstPlayerColorIndex, secondPlayerColorIndex, sizeBoardIndex, endOfCurrentData;
		try {
			reader = new BufferedReader(new FileReader(fileDes));
			String currentLine;
			while ((currentLine = reader.readLine()) != null) {
				// here for each line
				if ((firstPlayerIndex = currentLine.indexOf("First player is:")) >= 0) {
					if ((endOfCurrentData = currentLine.indexOf(".")) > 0) {
						String tempStartData = currentLine.substring(firstPlayerIndex + "First player is:".length() + 1, endOfCurrentData);
						// it should be as an integer string - who starts in the
						// game.
						GameData.playerThatStartInGame = Integer.parseInt(tempStartData);
					}
				}

				// read the first player color.
				if ((firstPlayerColorIndex = currentLine.indexOf("First player color is:")) >= 0) {
					if ((endOfCurrentData = currentLine.indexOf(".")) > 0) {
						GameData.firstPlayer = Integer.parseInt(currentLine.substring(firstPlayerColorIndex + "First player color is:".length() +1, endOfCurrentData));
					}
				}

				// read the second player color
				if ((secondPlayerColorIndex = currentLine.indexOf("Second player color is:")) >= 0) {
					if ((endOfCurrentData = currentLine.indexOf(".")) > 0) {
						GameData.secondPlayer = Integer.parseInt(currentLine.substring(secondPlayerColorIndex + "Second player color is:".length() + 1, endOfCurrentData));
					}
				}

				// read the size of the board
				if ((sizeBoardIndex = currentLine.indexOf("Size of the board:")) >= 0) {
					if ((endOfCurrentData = currentLine.indexOf(".")) > 0) {
						String temp = currentLine.substring(sizeBoardIndex + 1 + "Size of the board:".length(), endOfCurrentData);
						GameData.RowNumber = Integer.parseInt(temp);
						GameData.ColNumber = GameData.RowNumber;
					}
				}
			}
		} catch (FileNotFoundException e) {
			System.out.println("Problem into GameData::getDataFromText ");
		} catch (IOException io) {
			System.out.println("IOException into GameData::getDataFromText ");
		}
	}
}
