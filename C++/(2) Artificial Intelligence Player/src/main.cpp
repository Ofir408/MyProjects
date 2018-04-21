/*
 * main.cpp
 *
 *  Created on: Oct 25, 2017
 *     Names : Yuval Weinstein & Ofir Ben Shoham.
 *     Id: 208580613 & 208642496.
 */

#include <iostream>
#include "Board.h"
#include "Cell.h"
#include "GameLogic.h"
#include "CellsSwitcher.h"
#include <string>
#include "GameRunner.h"
#include "HumanPlayer.h"

using namespace std;

int main() {

	Board b;
	HumanPlayer humanPlayer('X'); // the first player is human player.
	HumanPlayer secondPlayer('O');
	AIPlayer aiSecondPlayer('O');
	char startPlayerSign = humanPlayer.getPlayerSign();
	GameRunner gr(b, humanPlayer, secondPlayer, startPlayerSign);


	int userChoice = GameRunner::menu();

	if (userChoice == 2) {
		gr.setSecondPlayer(aiSecondPlayer); // set the second player to AI.
	}

	b.printBoard();


	try {

		gr.run(); // run the game.
	} catch (exception *e) {
		cout << "exception was caught :( \n";
	}

	return 0;
}

