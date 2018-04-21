/*
 * GameRunner.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: ofir
 */

#include "GameRunner.h"
#include "HumanPlayer.h"
#include "AIPlayer.h"
#include "typeinfo"

namespace std {

GameRunner::GameRunner(Board b, Player& firstP, Player& secondPl,
		char startingPlayer) {
	board = b;
	GameLogic gameLogic(b);
	currentPlayer = startingPlayer;
	firstPlayer = &firstP;
	secondPlayer = &secondPl;
}

// come to this function with the knowledge that we can continue in the game.
// It means that at least for one player has a possible move to do.
void std::GameRunner::playNextMove(Player &playerCurrentTurn, bool isRemote) {

	if (isRemote) {

		this->currentPlayer = playerCurrentTurn.getPlayerSign();

		Board t = this->board;
		char signToSearchFor = playerCurrentTurn.getPlayerSign();
		vector<Cell> possibleCells = t.possibleCellsToAssign(signToSearchFor);
		if (possibleCells.empty()) {
			cout << "No Possible Moves for player: " << signToSearchFor
					<< " Current turn passes back to the other player.\n";
		}
		Cell userInput = playerCurrentTurn.chooseCell(&board,
				this->currentPlayer);
		this->board.inputAssignManager(this->currentPlayer, userInput);
		this->board.printBoard();
		return;
	}

	this->currentPlayer = playerCurrentTurn.getPlayerSign();

	Board t = this->board;
	char signToSearchFor = playerCurrentTurn.getPlayerSign();
	vector<Cell> possibleCells = t.possibleCellsToAssign(signToSearchFor);
	if (possibleCells.empty()) {
		cout << "No Possible Moves for player: " << signToSearchFor
				<< " Current turn passes back to the other player.\n";
	} else {
		Cell userInput = playerCurrentTurn.chooseCell(&board,
				this->currentPlayer);
		this->board.inputAssignManager(this->currentPlayer, userInput);
		this->board.printBoard();
	}

}

void std::GameRunner::run() {

	this->gameLogic = GameLogic(Board());

	while (canToContinue()) {
		Player &currentPlayer = getCurrentPlayerTurn();

		// check if it's instance of remote player, if yes send 		playNextMove(currentPlayer, true);
		// if it doesnt send 		playNextMove(currentPlayer, false);

		if (typeid(currentPlayer) == typeid(HumanPlayer)
				|| typeid(currentPlayer) == typeid(AIPlayer)) {
			playNextMove(currentPlayer, false);
			switchCurrentPlayer();
		} else {
			playNextMove(currentPlayer, true); // remote player.
			// after he made his turn, switch the current player.
		}

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
	bool temp = false;
	string lineInput;

	cout << " --- Welcome To Our Game --- \n";
	cout << "Ofir Ben-Shoham & Yuval Weinstein\n";
	cout << "Please choose your game option (Press 1 or 2 and enter): \n\n";
	cout << "1) If you want to play vs other Human Person\n";
	cout
			<< "2) If you want to play vs AI player ( But..he is very smart :) ) \n";
	cout << "3) If you want to play via a remove player\n";
	do {
		getline(std::cin, lineInput);
		temp = cin.fail() || lineInput.length() != 1
				|| atoi(lineInput.c_str()) > 3 || atoi(lineInput.c_str()) < 0;
		if (temp) {
			cout << "Doesn't legal chioce. Choose 1 or 2 or 3, then press enter"
					<< std::endl;
			cin.clear(); // reset the failed state
		}
	} while (temp);

	input = atoi(lineInput.c_str());

	if (input == 1) {
		cout << "\n No problem, You will play vs Human Player\n";
	} else if (input == 2) {
		cout << "\n No problem, You will play vs AI Player\n";
	} else if (input == 3) {
		cout << "\n No problem, You will play vs Remote Player\n";
	} else {
		cout << "Your choise doesn't legal, please choose 1 or 2 or 3\n";

	}
	return input;
}

string GameRunner::startOrJoinGame() {
	cout << "If you would like to start a new game please press 1" << endl;
	cout << "If you would like to join an existing game please press 2" << endl;
	cout << "If you would like to see the possible games to enter into, press 3"
			<< endl;
	string choice;
	cin >> choice;
	if (!choice.compare("1")) {
		return "start";
	} else if (!choice.compare("2")) {
		return "join";
	} else if (!choice.compare("3")) {
		return "list_games";
	} else {
		cout << "Your choice doesn't legal, choose 1 Or 2 Or 3 \n";
		return startOrJoinGame();
	}
}

string GameRunner::inputGameName() {
	cout << "enter the name of the game" << endl;
	string name;
	cin >> name;
	return name;
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

