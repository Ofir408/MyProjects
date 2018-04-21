/*
 * Main.cpp
 *
 *  Created on: Dec 2, 2017
 *     Names : Yuval Weinstein & Ofir Ben Shoham.
 *     Id: 208580613 & 208642496.
 */
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include "Server.h"
#include "ListGamesFunc.h"
#include "StartGameFunc.h"
#include "ThreadStarter.h"
#include "ThreadServer.h"
#include <pthread.h>

using namespace std;

int main() {

	int portNumber;
	string line;
	ifstream myfile("ConnectingDetails.txt");

	if (!myfile.is_open()) {
		cout << "Unable to open file";
		return -1;
	}
	// Otherwise, opened the file.

	getline(myfile, line); // no use of the ip adress in the server.

	getline(myfile, line);
	size_t posPort = line.find(":") + 1;
	if (posPort > 0) {
		string temp = line.substr(posPort);
		portNumber = atoi(temp.c_str());
	}
	myfile.close();

	GameManager gm;

	Server* server = new Server(portNumber, &gm); // the port we read from the file.
	server->socketInitialize();
	server->setAndStartServer();

	GameManager gameManager; // initialize game manager with the vector of all the games.
	ThreadServer threadServer(server);

	struct argsToThreadServer* argsToPassToThreadServer = new argsToThreadServer;
	argsToPassToThreadServer->gameManager = &gameManager;
	argsToPassToThreadServer->serverSocket = server->getServerSocket();
	argsToPassToThreadServer->secondClientSocket = -1; // by default, because we didn't do accept to the second client.
	argsToPassToThreadServer->shouldContinue =
			threadServer.getPointerToShouldContinue();
	argsToPassToThreadServer->server = server;

	pthread_t thread;
	// add the needed in the next line:
	int rc = pthread_create(&thread, NULL, ThreadServer::gamesRunner,
			argsToPassToThreadServer);

	if (rc) {
		cout << "error with creating thread" << endl;
	}
	try {
		threadServer.runServerLoop();
	} catch (const char* a) {
		threadServer.runServerLoop(); // continue the server also if client disconnected.
	}
	delete server;

	return 0;
}
