/*
 * CloseGameFunc.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: Ofir Ben-Shoham.
 */

#include "CloseGameFunc.h"

using namespace std;

CloseGameFunc::CloseGameFunc() {}


void CloseGameFunc::execute(vector<string> args, GameManager* gameManager) {
	// close the tread the has responssiblity of this game.

	// first get the name
	try {
		string nameOfGameToClose = args[2];
		int threadIndexToClose = gameManager->findGameAccordingString(nameOfGameToClose);
		gameManager->removeGame(nameOfGameToClose); // remove the game.
		gameManager->exitThread(nameOfGameToClose);


	} catch (exception* e ) {
		cout << "There is a problem into loseGameFunc::execute\n";
	}
}
