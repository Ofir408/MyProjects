/*
 * AIPlayer.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: ofir
 */

#include "AIPlayer.h"

namespace std {

/**
 * This function takes care to parts 2b and 2c in the pesudo code.
 * Gets:
 * vector<Cell> possibleCells - all the possible cells of the enemy in the
 * new board status (to make part 2b).
 * Return the maximum grade the the enemy can to get in the new board status.
 */
MiniMaxCell AIPlayer::getEnemyCurrentMaxGrade(Board &board,
		vector<Cell> enemyPossibleCells, char enemySign) {
	Board fakeBoard;
	int maxGrade = -1000, currentGrade;
	MiniMaxCell maxToReturn(Cell(-1, -1), -1); // just for initilize.
	for (unsigned int i = 0; i < enemyPossibleCells.size(); i++) {
		Cell currentCell = enemyPossibleCells[i];
		fakeBoard = board.copyConstructor(board);
		// put it for checking his grade if the enemy put his cell there.
		fakeBoard.inputAssignManager(enemySign, currentCell);
		//Cell currentCell = Cell(enemyPossibleCells[i].getX(),enemyPossibleCells[i].getY()) ;
		currentGrade = getEnemyCurrentGrade(fakeBoard, enemySign);
		if (currentGrade > maxGrade) {
			maxGrade = currentGrade;
			// update maxGradeCell to the max grade with the current cell
			maxToReturn.replace(currentCell, maxGrade);
		}
	}
	return maxToReturn;
}

/**
 *  put the possible cell in the board. Board doesn't change because this
 *  method gets (&) of the fake board (that changed) and not a pointer to our real board.
 *  now will check what is the grade of the enemy and returns this grade.
 */
int AIPlayer::getEnemyCurrentGrade(Board &board, char enemySign) {

	int enemyCells = board.howMuchCells(enemySign);
	char AIPlayerSign = Cell::returnOtherSign(enemySign);
	int AiGrade = board.howMuchCells(AIPlayerSign);
	// now, return the grade of the enemy:
	return enemyCells - AiGrade;

}

Cell AIPlayer::chooseCell(Board* board, char playerSign) {
	MiniMaxCell AINextMove = AIPlayer::returnNextMove(*board, playerSign);
	Cell c = AINextMove.getCellToPut();
	cout << "AI Played: (" << c.getX() + 1 << ", " << c.getY() + 1 << ")"
			<< endl;
	return c; // the cell that choosed by the AI player.
}

MiniMaxCell AIPlayer::returnNextMove(Board& board, char playerSign) {
	int lowestGrade = 1000;
	char enemySign = Cell::returnOtherSign(playerSign);
	MiniMaxCell lowestGradeToReturn;
	MiniMaxCell currentCellAndGrade;
	// just initialize to max when search for the min, so this initialize it doesn't matter

	// step 1 in pusudo code - finding all possible moves of the AI player
	vector<Cell> AIPossibleMoves = board.possibleCellsToAssign(playerSign);
	// for each possible moves of the AI player:
	for (unsigned int i = 0; i < AIPossibleMoves.size(); i++) {
		// part 2a, make the move in the memory (without any real change) but update the "fake" board.
		Board fakeBoard = board.copyConstructor(board);
		Cell cellToCheck = Cell(AIPossibleMoves[i].getX(),
				AIPossibleMoves[i].getY()); // dont change it!

		if (fakeBoard.isCellEmpty(cellToCheck)) {
			fakeBoard.enterToBoard(playerSign, AIPossibleMoves[i].getX(), // +1 REMOVED
					AIPossibleMoves[i].getY());
			// make switch if the fake board after the new cell that was placed.
			fakeBoard.inputAssignManager(playerSign, cellToCheck); // update the fake board.

			// Part 2b: checks the possibles moves of the enemy (not the AI player)
			// after the cell we added to fakeBoard
			vector<Cell> enemyPossibleMoves = fakeBoard.possibleCellsToAssign(
					enemySign);
			currentCellAndGrade.replace(
					AIPlayer::getEnemyCurrentMaxGrade(fakeBoard,
							enemyPossibleMoves, enemySign));
			if (currentCellAndGrade.getGradeForThisCell() < lowestGrade) {
				// swap the lowest grade
				lowestGrade = currentCellAndGrade.getGradeForThisCell();
				// swap the lowest object with the lowest grade for the enemy next move.
				// need the AI moves that makes the enemy to have minimal grade.
				lowestGradeToReturn.replace(
						MiniMaxCell(AIPossibleMoves[i], lowestGrade));
			}
		}
	}

	return lowestGradeToReturn;
}

} /* namespace std */

