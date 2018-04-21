/*
 * ThreadServer.cpp
 *
 *  Created on: Dec 28, 2017
 *      Author: Ofir Ben-Shoham.
 */

#include "ThreadServer.h"
#include "StructDef.h"
#include "Server.h"
#include "ThreadStarter.h"

ThreadServer::ThreadServer(Server* server) {
	this->shouldContinue = new bool();
	*this->shouldContinue = true;
	this->server = server;
}

void* ThreadServer::gamesRunner(void* input) {

	struct argsToThreadServer *gameDes = (struct argsToThreadServer *) input;
	struct argsToThreadServer gameDescription; //gameDescription to save

	gameDescription.firstClientSocket = gameDes->firstClientSocket;
	gameDescription.secondClientSocket = gameDes->secondClientSocket;
	gameDescription.gameManager = gameDes->gameManager;
	gameDescription.serverSocket = gameDes->serverSocket;
	gameDescription.shouldContinue = gameDes->shouldContinue;
	gameDescription.server = gameDes->server;
	delete gameDes; //free the memory allocated

	// cin to get an input from the server.
	// Because we want the abillity to close all the games & the server using command "end" from the server main runner.
	string gettingInput = " ";

	while (true) {
		cin >> gettingInput;
		if (gettingInput.compare("exit") == 0) {
			cout
					<< "I got exit from the user. Prepering to close all the games & sockets\n";
			break; // exit from the thread, and return to the main to close all the needed things.
		}
	}

	*gameDescription.shouldContinue = false;
	// close the thread
	cout << "exit from the thread\n";

	GameManager * temp = gameDescription.server->getGamesManager();
	ThreadServer::exitFromAllTheGames(temp,  gameDescription.server);
	// finally, exit from the main of the server


}

void ThreadServer::runServerLoop() {
	struct passedArguments {
		int firstClientSocket;
		int secondClientSocket;
		GameManager* gameManager;
	};

	struct passedArguments argsToPass;
	argsToPass.gameManager = this->server->getGamesManager();

	string gettingInput = "";
	//expecting new connections all the time
	cout << "server is ready for new clients" << endl;
	while (*this->shouldContinue == true) {
		argsToPass.firstClientSocket = server->acceptClientSocket(
				server->getServerSocket());
		argsToPass.secondClientSocket = -1; // by default, because we didn't do accept to the second client.
		pthread_t thread;
		pthread_create(&thread, NULL, ThreadStarter::readAndRunCommand, &argsToPass);
	}
}

void ThreadServer::exitFromAllTheGames(GameManager* gameMng, Server* server) {
	cout << "exitFromAllTheGames was called\n";
	// first, pass on the games
	//GameManager* gameMng = this->server->getGamesManager();
	vector<GameDescriptor*> gamesToRemove = gameMng->getGamesDesList();
	unsigned int index = 0;
	for (index = 0; index < gamesToRemove.size(); index++) {
		// remove each game
		GameDescriptor* d = gamesToRemove[index];
		string currentGameName = d->getNameOfGame();

		// close the thread of the game and remove it from the vector<GameDescriptor>:
		gameMng->removeGame(currentGameName);

		// the socket are closed into the games itself, since they get -100 as an input cell.
		// now, after closing the clients sockets we want to close the thread of this game

	}

	close(server->getServerSocket());
	exit(0); // exit from the main of the server.
}

bool* ThreadServer::getPointerToShouldContinue() {
	return this->shouldContinue;
}
