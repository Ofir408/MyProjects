/*
 * GameLogic.cpp
 *
 *  Created on: Nov 4, 2017
 *      Author: Ofir Ben shoham
 *      Id: 208642496.
 */

#include "GameLogic.h"

namespace std {

GameLogic::GameLogic(Board newBoard) {
	this->board = newBoard;
}

bool GameLogic::canAssign(char player, Cell cellToCheck) {
	//vector<Cell> possibleMovesVec = possibleCellsToAssign(board, player);
	vector<Cell> possibleMovesVec = board.possibleCellsToAssign(player);

	for (unsigned i = 0; i < possibleMovesVec.size(); i++) {
		Cell currentCell = Cell(possibleMovesVec[i].getX(),
				possibleMovesVec[i].getY());
		if (currentCell.compareCells(cellToCheck)) {
			return true;
		}
	}
	return false;
}

void GameLogic::printPossibleCells(char player) {
	vector<Cell> possibleMovesVec = board.possibleCellsToAssign(player);
	cout << "Player: " << player << " ,Your Possible Moves are: ";
	for (unsigned i = 0; i < possibleMovesVec.size(); i++) {
		possibleMovesVec[i].printCell();
	}
	cout
			<< " \nPlease choose a cell from your possible options, for example write 3,4:"
			<< endl;
}

// check this method with testers...
void GameLogic::inputAssignManager(char player, Cell input) {
	// assume that the input is valid for this player.
	if (canAssign(player, input)) {
		board.rowSequenceCheck(player, input, true);
		board.colSequenceCheck(player, input, true);
		board.slantSequenceCheck(player, input, true);
	}
}

}
