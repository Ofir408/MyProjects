
public class GameRunner {
	/*
	 * GameRunner
	 *
	 *  Created on: Nov 9, 2017
	 *      Author: Ofir Ben-Shoham.
	 */

	private Board board;
	private GameLogic gameLogic;
	private char currentPlayer;

	
	/**
	 * Constructor of GameRunner.
	 * Gets:
	 * Board b - a board that changes in our game.
	 * char player - the player who starts the game.
	 */
	GameRunner(Board b, char player) {
		this.board = b;
		this.gameLogic = new GameLogic(b);
		this.currentPlayer = player;
	}


	/**
    * come to this function with the knowledge that we can continue in the game.
	* It means that at least for one player has a possible move to do.
	* returns the next player that should play.
    **/
	char playNextMove(int x, int y) {
		Cell userInput = new Cell(x, y, this.currentPlayer); // gets input from the user
		boolean turnMade = gameLogic.inputAssignManager(this.currentPlayer, userInput);
		this.board = gameLogic.getBoard();
		// after he made his turn, switch the current player.
		if (!this.canContinue()) {
			return ' ';
		}
		if (turnMade) {
			this.switchCurrentPlayer();
		}
		if (!this.gameLogic.hasPossibleMoves(this.currentPlayer)) {
			this.switchCurrentPlayer();
			return this.currentPlayer;
		}
		return this.currentPlayer;
	}
	
	/**
	 * return true if we can continue in the game.
	 * Otherwise -> return false.
	 */
	boolean canContinue() {
			// The game is ended when all the board is full or no more moves
			// is possible for the players.
			char before = this.currentPlayer; // the first.
			switchCurrentPlayer();
			char after = this.currentPlayer;  // the second.
			boolean b1 = board.possibleCellsToAssign(before).isEmpty();
			boolean b2 = board.possibleCellsToAssign(after).isEmpty();

			if (this.board.isBoardFull() || (b1 && b2)) {
				this.currentPlayer = before;
				return false; // need to stop.
			}
			this.currentPlayer = before;
			return true; // otherwise, can to continue in the game.
	}

	/**
	 * switch the current player after the second made his move.
	 * It means, change the currentPlayer field into GameRunner.
	 */
	void switchCurrentPlayer() {
		if (currentPlayer == 'X') {
			this.currentPlayer = 'O';
		} else {
			currentPlayer = 'X'; // because it was 'O'.
		}
	}
	
}