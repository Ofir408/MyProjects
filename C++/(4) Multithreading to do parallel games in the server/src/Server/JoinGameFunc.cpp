/*
 * JoinGameFunc.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: Ofir Ben-Shoham
 */

#include "JoinGameFunc.h"
#include <pthread.h>
#include "ThreadStarter.h"

using namespace std;

JoinGameFunc::JoinGameFunc(void* (*funcToStart)(void*)) {
	this->funcToStart = funcToStart;
}

// args[0]- socket of the client that wants to enter to
// args[2] game (name of game).
void JoinGameFunc::execute(vector<string> args, GameManager* gameManager) {
	// first check if we really have a game in this name.

	try {
		string secondClientSocket = args[0];
		const char* socketTemp = secondClientSocket.c_str();
		int secondClientSocketNumber = atoi(socketTemp);
		if (secondClientSocketNumber <= 0) {
			cout << "Failed to prase the number in ListGamesFunc::execute() \n";
			exit(-1);
		}

		// now, want to create new game according args[2]
		string newGameName = args[2];
		if (gameManager->findGameAccordingString(newGameName) < 0) {
			// we don't have a game it this name.
			int n = write(secondClientSocketNumber, "-1", sizeof("-1"));
			if (n <= 0) {
				cout
						<< "Can't send to the client that he can't choose this game name, into JoinGameFunc::execute\n";
				exit(-1);
			}
		} else {
			int n = write(secondClientSocketNumber, "0", sizeof("0"));
			if (n <= 0) {
				cout
						<< "Can't send to the client that he can choose this game name, into JoinGameFunc::execute\n";
				exit(-1);
			}
			// the second player can enter to the game.
			struct passedArguments* argsToPass = new struct passedArguments();
			GameDescriptor* toEnterIntoHim =
					gameManager->getGameAccordingString(newGameName);
			toEnterIntoHim->setSecondClientSocket(secondClientSocketNumber); // enter the socket of the second player.

			argsToPass->firstClientSocket =
					toEnterIntoHim->getFirstClientSocket();
			argsToPass->secondClientSocket =
					toEnterIntoHim->getSecondClientSocket();

			// write to the clients that they can start in the game.

			argsToPass->gameManager = gameManager;

			if (!toEnterIntoHim->IsBothPlayersConnects()) {
				/*cout
				 << "Not both the players are connected, JoinGameFunc::execute\n";
				 exit(-1);*/
				cout
						<< "(-) Doesn't call to workWithClients thread, check sockets value into JoinGameFunc::execute \n";
			} else {
				// now we know that we have two players into the game, so we can start it,
				// therefore, enter a tread.

				/*
				 * We have two players into the game, therefore I want to open new thread to run the game.
				 * the function that should run the game is workWithClients() into Server.
				 * The object that need to be send into workWithClients() is toEnterIntoHim that contains
				 * data about the name of the game and the sockets of the two clients.
				 */


				pthread_t thread;
				// add the needed in the next line:
				int rc = pthread_create(&thread, NULL,
						ThreadStarter::workWithClients, (void*) argsToPass);

				if (rc) {
					cout << "error with creating thread" << endl;
				}
				gameManager->setThread(thread, toEnterIntoHim->getNameOfGame());
			}
		}

	} catch (exception * e) {
		cout
				<< "Problem with prasing the input agrs, into StartGameFunc::execute\n";
		exit(-1); // can't continue ...
	}

}
