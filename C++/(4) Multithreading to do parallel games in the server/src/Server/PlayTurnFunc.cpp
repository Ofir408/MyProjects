/*
 * PlayTurnFunc.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: ofir
 */

#include "PlayTurnFunc.h"
#include <iostream>

using namespace std;

PlayTurnFunc::PlayTurnFunc() {
}

void PlayTurnFunc::execute(vector<string> args, GameManager* gameManager) {
	int firstClientSokcet, secondClientSocket, x, y;

	// get from args[0] the socket number of the first client.
	string firstSocketString = args[0];
	const char* socketTemp = firstSocketString.c_str();
	firstClientSokcet = atoi(socketTemp);
	if (firstClientSokcet <= 0) {
		return;
	}

	// the same, for args[1] and the second client.
	string secondSocketString = args[1];
	const char* secondSocketTemp = secondSocketString.c_str();
	secondClientSocket = atoi(secondSocketTemp);
	if (secondClientSocket <= 0) {
		return;
	}

	// get from args[2] the x value of the cell that the first player choose.
	string xString = args[2];
	const char* xStringTemp = xString.c_str();
	x = atoi(xStringTemp);
	if (x <= 0) {
		return;
	}

	// get from args[3] the y value of the cell that the first player choose.
	string yString = args[3];
	const char* yStringTemp = yString.c_str();
	y = atoi(yStringTemp);
	if (y <= 0) {
		return;
	}

	// send it to the second socket client (secondClientSocket) as follows:
	int n = write(secondClientSocket, &x, sizeof(x));
	if (n <= 0) {
		cout << "\n\n--- The client wanted to finish the game --- \n";
		close(firstClientSokcet);
		close(secondClientSocket);
		exit(0);
	}
	if (write(secondClientSocket, &y, sizeof(y)) <= 0) {
		cout << "The client wanted to finish the game\n";
		close(firstClientSokcet);
		close(secondClientSocket);
		exit(0);
	}

}

void* std::PlayTurnFunc::threadPlayExcute(void* playSt) {
	playStruct *p = (playStruct*) playSt;

	int firstClientSokcet, secondClientSocket, x, y;
	usleep(500);
	vector<string> args = p->args;

	// get from args[0] the socket number of the first client.
	string firstSocketString = args[0];
	const char* socketTemp = firstSocketString.c_str();
	firstClientSokcet = atoi(socketTemp);
	if (firstClientSokcet <= 0) {
		return 0;
	}

	// the same, for args[1] and the second client.
	string secondSocketString = args[1];
	const char* secondSocketTemp = secondSocketString.c_str();
	secondClientSocket = atoi(secondSocketTemp);
	if (secondClientSocket <= 0) {
		return 0;
	}

	// get from args[2] the x value of the cell that the first player choose.
	string xString = args[2];
	const char* xStringTemp = xString.c_str();
	try {
		x = atoi(xStringTemp);
	} catch (exception *e) {
		return 0;
	}
	// get from args[3] the y value of the cell that the first player choose.
	string yString = args[3];
	const char* yStringTemp = yString.c_str();
	y = atoi(yStringTemp);
	if (y <= 0) {
		return 0;
	}

	// send it to the second socket client (secondClientSocket) as follows:
	write(secondClientSocket, &x, sizeof(x));

	if (write(secondClientSocket, &y, sizeof(y)) <= 0) {
		cout << "The client wanted to finish the game\n";
		close(firstClientSokcet);
		close(secondClientSocket);
		exit(0);
	}
	return (void*)1;
}
