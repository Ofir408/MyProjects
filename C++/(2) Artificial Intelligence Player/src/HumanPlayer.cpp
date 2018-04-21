/*
 * HumanPlayer.cpp
 *
 *  Created on: Nov 24, 2017
 *      Author: Ofir Ben Shoham
 */

#include "HumanPlayer.h"

namespace std {

Cell HumanPlayer::chooseCell(Board* board, char playerSign) {
	printPossibleMoves(*board);
	Cell userInput = getUserInput(*board); // gets input from the user
	return userInput;
}

Cell HumanPlayer::askInput() {
	string lineInput; //, secondInput;
	getline(cin, lineInput);
	size_t temp = lineInput.find(",");
	string r = lineInput.substr(0, temp);
	string s = lineInput.substr(temp + 1);
	int inputRow = atoi(r.c_str());
	int inputCol = atoi(s.c_str());
	Cell inputCell(inputRow, inputCol);
	return inputCell;
}

void HumanPlayer::printPossibleMoves(Board& b) {
	vector<Cell> possibleMovesVec = b.possibleCellsToAssign(playerSign);
	cout << "Player: " << playerSign << " ,Your Possible Moves are: ";
	for (unsigned i = 0; i < possibleMovesVec.size(); i++) {
		possibleMovesVec[i].printCell();
	}
	cout << " \nPlease choose a cell from your possible options,"
			" for example write 3,4:" << endl;
}

bool HumanPlayer::checkValidInput(Board &b, Cell inputCell) {
	return inputCell.isEmptyCell() && inputCell.isValid()
			&& b.canAssign(this->playerSign, inputCell);
}

Cell HumanPlayer::getUserInput(Board &b) {
	Cell inputCell = askInput();

	if (!checkValidInput(b, inputCell)) {
		do {
			if (!b.canAssign(this->playerSign, inputCell)) {
				cout << "\nYou didn't choose from your options.\n";
			}
			cout << "Your choice doesn't legal\n";

			printPossibleMoves(b);

			inputCell = askInput();
		} while (!checkValidInput(b, inputCell));
	}
	return inputCell;
}

} /* namespace std */
