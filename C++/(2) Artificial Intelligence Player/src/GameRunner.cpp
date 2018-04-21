/*
 * GameRunner.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: ofir
 */

#include "GameRunner.h"

namespace std {

GameRunner::GameRunner(Board b, Player& firstP, Player& secondPl,
		char startingPlayer) {
	board = b;
	GameLogic gameLogic(b);
	currentPlayer = startingPlayer;
	firstPlayer = &firstP;
	secondPlayer = &secondPl;
}

// come to this function with the knowlenge that we can continue in the game.
// It means that at least for one player has a possible move to do.
void std::GameRunner::playNextMove(Player &playerCurrentTurn) {
	this->currentPlayer = playerCurrentTurn.getPlayerSign();

	if (board.possibleCellsToAssign(this->currentPlayer).empty()) {
		string userPassesChecking;
		cout << "\n No Possible Moves. Play passes back to the other player.\n "
				"Press any key to continue and then press on Enter.\n";
		cin >> userPassesChecking;
		if (userPassesChecking.length() != 0) {
			// switch the turn to the other player.
			cout
					<< "\nOkay, Your turn was passed to the other player, that has moves\n";
			cin.ignore();
			return;
			Cell userInput = playerCurrentTurn.chooseCell(&board,
					currentPlayer); // gets input from the user
			gameLogic.inputAssignManager(this->currentPlayer, userInput);
			this->board = *gameLogic.getBoard();
			this->board.printBoard();
		}
	} else {

		Cell userInput = playerCurrentTurn.chooseCell(&board,
				this->currentPlayer);
		gameLogic.inputAssignManager(this->currentPlayer, userInput);
		this->board = *gameLogic.getBoard();
		this->board.printBoard();
	}
}

void std::GameRunner::run() {

	this->gameLogic = GameLogic(Board());
	while (canToContinue()) {
		Player &currentPlayer = getCurrentPlayerTurn();
		playNextMove(currentPlayer);
		// after he made his turn, switch the current player.
		this->switchCurrentPlayer();

	}
	// print who won.
	this->board.whoWon();
}

bool std::GameRunner::canToContinue() {
	// The game is ended when all the board is full or no more moves
	// is possible for the players.
	char before = this->currentPlayer; // the first.
	switchCurrentPlayer();
	char after = this->currentPlayer;  // the second.
	bool b1 = board.possibleCellsToAssign(before).empty();
	bool b2 = board.possibleCellsToAssign(after).empty();

	if (this->board.isBoardFull() || (b1 && b2)) {
		this->currentPlayer = before;
		return false; // need to stop.
	}
	this->currentPlayer = before;
	return true; // otherwise, can to continue in the game.

}

int GameRunner::menu() {
	int input;
	string lineInput;

	cout << " --- Welcome To Our Game --- \n";
	cout << "Ofir Ben-Shoham & Yuval Weinstein\n";
	cout << "Please choose your game option (Press 1 or 2 and enter): \n\n";
	cout << "1) If you want to play vs other Human Person\n";
	cout
			<< "2) If you want to play vs AI player ( But..he is very smart :) ) \n";
	do {
		getline(std::cin, lineInput);
		if (cin.fail() || lineInput.length() != 1) {
			cout << "Doesn't legal chioce. Choose 1 or 2, then press enter"
					<< std::endl;
			cin.clear(); // reset the failed state
		}
	} while (cin.fail() || lineInput.length() != 1);

	input = atoi(lineInput.c_str());

	if (input == 1) {
		cout << "\n No problem, You will play vs Human Player\n";
	} else {
		cout << "\n No problem, You will play vs AI Player\n";
	}
	return input;
}

void std::GameRunner::switchCurrentPlayer() {
	if (currentPlayer == 'X') {
		this->currentPlayer = 'O';
	} else {
		currentPlayer = 'X'; // because it was 'O'.
	}
}
}

Player& std::GameRunner::getCurrentPlayerTurn() {
	if (currentPlayer == 'x' || currentPlayer == 'X') {
		return *firstPlayer; // in order to prevent from changing it.
	}
	return *secondPlayer; // safer passing by & and not a pointer that can be changed.
}

