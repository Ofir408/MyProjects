
import java.util.Vector;


public class GameLogic {
	/*
	 * GameLogic
	 *
	 *  Created on: Nov 4, 2017
	 *      Author: Ofir Ben shoham
	 */

	private Board board; 
	
	GameLogic(Board newBoard) {
		this.board = newBoard;
	}
	
	/**
	 * This function gets:
	 * char current player -> X / O.
	 * Cell emptyCellToCheck -> an empty cell that we check.
	 * returns true if the player can assign his sign in the input empty cell.
	 * Otherwise returns false.
	 */
	boolean canToAssign(Board board, char player, Cell emptyCellToCheck)  {
		return board.canToAssign(player, emptyCellToCheck);
	}

	/**
	 * returns vector of cells that each one represents an empty cell that
	 * the current player can assign his sign there.
	 * each cell from the return vector is from (1..8, 1..8)
	 * Because we want to print the possible moves
	 * to the current player according how the board is printed and not from (0..7) as the array works.
	 *
	 */
	Vector<Cell> possibleCellsToAssign(Board board, char player)  {
		Vector<Cell> vecToReturn = new Vector<Cell>();
		// passing on the all cells in our board.
		for (int r = 1; r <= GameData.RowNumber; r++) {
			for (int c = 1; c <= GameData.ColNumber; c++) {
				Cell currentCell = new Cell(r, c, ' ');
				if (board.isCellEmpty(currentCell)
						&& canToAssign(board, player, currentCell)) {
					// add it to our vector with +1 because we want to show to our user
					// the possible cells from (1..8, 1..8) and not from (0..7, 0..7).

					vecToReturn.addElement(	new Cell(currentCell.getX() + 2,
							currentCell.getY() + 2, ' ')); // +2 because we lower twice when we created Cell using its constructor.);(
						
				}
			}
		}
		return vecToReturn;
	}
	
	/**
	 * gets char of the current player (X / O) and returns true if he has possible moves.
	 * Otherwise-> false.
	 */
	boolean hasPossibleMoves(char player)  {
		return !possibleCellsToAssign(board, player).isEmpty();
	}

	/**
	 * checks if a sign can be inserted in a specific cell.
	 * @param player is the sign to check
	 * @param cellToCheck is the cell we want to insert the player into
	 * @return true if the insertion is possible and false otherwise
	 */
	boolean canAssign(char player, Cell cellToCheck)  {
		Vector<Cell> possibleMovesVec = possibleCellsToAssign(board, player);

		for (int  i = 0; i < possibleMovesVec.size(); i++) {
			Cell currentCell = new Cell(possibleMovesVec.get(i).getX(),
					possibleMovesVec.get(i).getY(), ' ');
			if (currentCell.compareCells(cellToCheck)) {
				return true;
			}
		}
		return false;
	}
	
	/**
	 * getter for board.
	 * @return the board of game logic
	 */
	Board getBoard() { return board; }
	
	

	/**
	 * char player -> X\O, the sign of the current player.
	 * Cell input  -> the input that the player wrote,
	 * here we assume that his choose
	 * is valid.
	 */
	boolean inputAssignManager(char player, Cell input) {
		// assume that the input is valid for this player.
		if (canAssign(player, input)) {
			board.rowSequenceCheck(player, input, true);
			board.colSequenceCheck(player, input, true);
			board.slantSequenceCheck(player, input, true);
			return true;
		} else {
			return false;
		}
	}

}
