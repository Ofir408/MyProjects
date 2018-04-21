/*
 * Client.cpp
 *
 *  Created on: Dec 6, 2017
 *      Author: Ofir Ben Shoham.
 */

#include "Client.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>

using namespace std;

Client::Client(const char *serverIP, int serverPort) :
		serverIP(serverIP), serverPort(serverPort), clientSocket(0) {
	cout << "Client" << endl;
	playerNum = 0; // by default
}

void Client::connectToServer() {
	// Create a socket point
	clientSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (clientSocket == -1) {
		throw "Error opening socket";
	}
	// Convert the ip string to a network address
	struct in_addr address;
	if (!inet_aton(serverIP, &address)) {
		throw "Can't parse IP address";
	}

	// Get a hostent structure for the given host address
	struct hostent *server;
	server = gethostbyaddr((const void *) &address, sizeof address, AF_INET);
	if (server == NULL) {
		throw "Host is unreachable";
	}
	// Create a structure for the server address
	struct sockaddr_in serverAddress;
	bzero((char *) &address, sizeof(address));
	serverAddress.sin_family = AF_INET;
	memcpy((char *) &serverAddress.sin_addr.s_addr, (char *) server->h_addr, server->h_length);
	// htons converts values between host and network byte orders
	serverAddress.sin_port = htons(serverPort);
	// Establish a connection with the TCP server
	if (connect(clientSocket, (struct sockaddr *) &serverAddress,
			sizeof(serverAddress)) == -1) {
		throw "Error connecting to server";
	}
	cout << "Connected to server" << endl;
}

DoubleCell Client::sendAndWriteToServer(Board* board) {
	int x, y;
	Cell readenCell;
	Cell myCell;


	if (playerNum == 1) {

		if (board->canContinue('X')) {
			cout << "Enter x\n";
			cin >> x;
			cout << "Enter y\n";
			cin >> y;
			myCell = Cell(x, y, 'X');
			if (!board->canAssign('X', myCell)) {
				cout << "You choise doesn't legal, please choose again\n";
				board->printPossibleCells('X');
				return sendAndWriteToServer(board);
			}

			board->inputAssignManager('X', myCell);
			board->printBoard();

			sendTurnToServer(x, y);
		} else {
			sendTurnToServer(4, 4);
			cout << "Here (1)\n";
		}

		if (board->canContinue('O')) {
			readenCell = getCurrentTurn(); // then he reads.
			readenCell.setSign('O');
			cout << "your opponent played: (" << readenCell.getX() + 1 << ", "
					<< readenCell.getY() + 1 << ")\n";
			board->inputAssignManager('O', readenCell);
		}

	} else {
		if (board->canContinue('X')) {
			readenCell = getCurrentTurn(); // first he reads.
			readenCell.setSign('X');
			board->inputAssignManager('X', readenCell);
			cout << "your opponent played: (" << readenCell.getX() + 1 << ", "
					<< readenCell.getY() + 1 << ")\n";
			board->printBoard();
		}

		if (board->canContinue('O')) {
			board->printPossibleCells('O');
			cout << "Enter x\n";
			cin >> x;
			cout << "Enter y\n";
			cin >> y;

			myCell = Cell(x, y, 'O');
			while (!board->canAssign('O', myCell)) {
				cout << "You choise doesn't legal, please choose again\n";
				board->printPossibleCells('O');
				cout << "Enter x\n";
				cin >> x;
				cout << "Enter y\n";
				cin >> y;
				myCell = Cell(x, y, 'O');
			}
			sendTurnToServer(x, y);
			myCell = Cell(x, y, 'O');
		} else {
			sendTurnToServer(0, 0);
			cout << "Here (2)\n";

		}
	}
	this->turnsToPlay.setOtherPlayerMove(readenCell);
	this->turnsToPlay.setCurrentPlayerMove(myCell);

	// add here the print of the board.

	DoubleCell toReturn(readenCell, myCell);
	return toReturn;
}

void Client::readHisCilentNum() {
	int clientNumber;
	int r = read(clientSocket, &clientNumber, sizeof(clientNumber));
	if (r < 0) {
		throw "Error reading from the socket in sendToServer() ";
	}
	cout << "*** My Client id is: " << clientNumber << " ***\n";
	if (clientNumber == 1) {
		this->playerNum = 1;
	} else {
		this->playerNum = 2;
	}
}

Cell Client::getCurrentTurn() {
	int row = 0;
	int col = 0;

	int n = read(clientSocket, &row, sizeof(row));
	cout << "passed\n";
	if (n <= 0) {
		// it means that we need to exit from the game
		close(this->clientSocket);
		cout << endl << endl
				<< "Dear Player, this client were asked to finish the game since the other clients wanted, or the server"
						".\n I hope that you enjoyed to play into our game.. As a little favor,  "
				<< "Please give us a grade that will make us happy :) \n";
		exit(0);
	}

	if (read(clientSocket, &col, sizeof(col)) <= 0) {
		// it means that we need to exit from the game
		close(this->clientSocket);
		cout << endl << endl
				<< "Dear Player, this client were asked to finish the game since the other clients wanted, or the server"
						".\n I hope that you enjoyed to play into our game.. As a little favor,  "
				<< "Please give us a grade that will make us happy :) \n";
		exit(0);
	}
	Cell c(row, col);
	return c;
}

void Client::sendTurnToServer(int row, int col) {

	char turnCommand[50] = "play";
	string x = convertIntToString(row);
	string y = convertIntToString(col);

	//creating the message according to the right format ("play<row><>col")
	strcat(turnCommand, " <");
	strcat(turnCommand, x.c_str());
	strcat(turnCommand, ">");
	strcat(turnCommand, "<");
	strcat(turnCommand, y.c_str());
	strcat(turnCommand, ">");

	// Write the point arguments to the socket
	int n = write(clientSocket, turnCommand, sizeof(turnCommand));
	if (n <= 0) {
		close(this->clientSocket);
		cout << endl << endl
				<< "Dear Player, this client were asked to finish the game since the other clients wanted, or the server"
						".\n I hope that you enjoyed to play into our game.. As a little favor,  "
				<< "Please give us a grade that will make us happy :) \n";
		exit(0);
	}
}

string std::Client::convertIntToString(int i) {
	ostringstream temp;
	temp << i;
	return temp.str();
}

int Client::makeFirstMessage(string command, string gameName) {
	char message[70];
	//creating the message according to the right format ("command<gameNamge>")
	message[0] = 0;
	strcat(message, command.c_str());
	strcat(message, " <");
	strcat(message, gameName.c_str());
	strcat(message, ">");

	//sending the message
	int n = write(clientSocket, message, sizeof(message));
	if (n < 0) {
		throw "Error sending client's first message";
	}

	n = read(clientSocket, message, sizeof(message));
	if (n < 0) {
		throw "Error receiving server's first message";
	}
	if (strcmp(message, "0") == 0) {
		return 0;
	} else {
		return -1;
	}
}

string Client::askForGameList() {
	char message[70] = "list_games";
	char games[70];
	//sending the message
	int n = write(clientSocket, message, sizeof(message));
	if (n < 0) {
		throw "Error sending client's list games command";
	}
	n = read(clientSocket, games, sizeof(games));
	if (n < 0) {
		throw "Error receiving client's list games command";
	}
	return string(games);
}

void Client::setTurnByCommand(string command) {
	if (command.compare("start") == 0) {
		this->playerNum = 1;
	} else if (command.compare("join") == 0) {
		this->playerNum = 2;
	}
}

