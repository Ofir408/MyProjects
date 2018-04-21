/*
 * Server.cpp
 *
 *  Created on: Dec 2, 2017
 *      Author: Ofir Ben Shoham.
 */

#define maxConnections 10
#include "Server.h"
#include <stdio.h>
#include <stdlib.h>

using namespace std;

Server::Server(int portNum, GameManager* gm) {
	this->serverPortNumber = portNum;
	this->serverSocket = 0; // by default, will change after initalize.
	this->gamesManager = gm;
	this->threadPool = new ThreadPool(5);
}

void Server::socketInitialize() {
	this->serverSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (this->serverSocket < 0) {
		perror("Failed in socketInitialize() \n");
	}
}

void Server::setAndStartServer() {
	// Assign a local address to the socket
	struct sockaddr_in serverAddress;
	// now, to be ensure that all his data is empty.
	memset(&serverAddress, 0, sizeof(serverAddress));

	// initialize the serverAdress as followsthrow "Error on binding";
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_addr.s_addr = INADDR_ANY;
	serverAddress.sin_port = htons(this->serverPortNumber);

	// bind according the serverSocket that we built & the serverAdress.
	try {
		if (bind(serverSocket, (struct sockaddr *) &serverAddress,
				sizeof(serverAddress)) < 0) {
			cout
					<< "*** Please Close all the clients before opening new server *** \n";
			cout << "Then you will be able to start new client\n";
			cout
					<< "This is because they are waiting to their move before closing their game after the server exits.\n";
			exit(0);
		}
	} catch (const char* c) {
		cout << "Please replace the port number into the settings file\n";
	}

	// Start listening to incoming connections
	listen(serverSocket, maxConnections);
}

void Server::closeServer() {
	close(this->serverSocket);
}

void Server::GetAndSendIntsToClient(int clientSocToGetFrom,
		int clientSocToSend) {
	int rowNum = 0, colNum = 0;

	//while (true) {
	rowNum = 0;
	colNum = 0;
	int n = read(clientSocToGetFrom, &rowNum, sizeof(rowNum)); // read the first arg.
	if (n <= 0) {
		cout << "Can't get the first number" << endl;
	}

	n = read(clientSocToGetFrom, &colNum, sizeof(colNum)); // read the second arg.
	if (n <= 0) {
		cout << "Can't get the second number" << endl;
	}

	// now write
	sendTurn(clientSocToSend, rowNum, colNum);
}

void Server::sendTurn(int clientSocToWriteInto, int row, int col) {
	int n = write(clientSocToWriteInto, &row, sizeof(row));
	if (n <= 0) {
		cout
				<< "n\nThe server asked to be closed since one from the clients exited\n";
		close(this->getServerSocket());
		exit(0);
	}
	n = write(clientSocToWriteInto, &col, sizeof(col));
	if (n <= 0) {
		cout
				<< "n\nThe server asked to be closed since one from the clients exited\n";
		close(this->getServerSocket());
		exit(0);
	}
}

int Server::acceptClientSocket(int socketOfServer) {
	int clientSocket;
	struct sockaddr_in clientAddress;
	socklen_t clientAddressSize;

	try {
		// Accept a new client connection for the first client.
		clientSocket = accept(socketOfServer,
				(struct sockaddr *) &clientAddress, &clientAddressSize);

		if (clientSocket < 0) {
			cout << "no more clients to connect" << endl;
		}
		return clientSocket;
	} catch (const char* a) {
		//acceptClientSocket(socketOfServer);
		cout << "Please change the port number into the settings file\n";
	}
	return clientSocket;
}

void Server::startGameByGameDescriptor(string name, int firstClientSock,
		int secondClientSock) {
	this->gamesManager->addGame(name, firstClientSock, secondClientSock);
}

string Server::convertIntToString(int i) {
	ostringstream temp;
	temp << i;
	return temp.str();
}
