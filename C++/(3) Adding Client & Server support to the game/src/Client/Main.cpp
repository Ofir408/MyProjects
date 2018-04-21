/*
 * Main.cpp
 *
 *  Created on: Dec 2, 2017
 *      Author: Ofir Ben Shoham
 */
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include "Client.h"
#include "Board.h"
#include "GameRunner.h"
#include "HumanPlayer.h"
#include "RemotePlayer.h"

using namespace std;

int main() {

	Board b;
	int userChoice = GameRunner::menu();
	if (userChoice == 1 || userChoice == 2) {
		HumanPlayer humanPlayer('X'); // the first player is human player.
		HumanPlayer secondPlayer('O');
		AIPlayer aiSecondPlayer('O');
		char startPlayerSign = humanPlayer.getPlayerSign();
		GameRunner gr(b, humanPlayer, secondPlayer, startPlayerSign);
		if (userChoice == 2) {
			gr.setSecondPlayer(aiSecondPlayer); // set the second player to AI.
		}

		b.printBoard();

		try {

			gr.run(); // run the game.
		} catch (exception *e) {
			cout << "exception was caught :( \n";
		}

	} else {

		// remote player
		// first read the data from the file
		const char* ipAdress;
		int portNumber;
		string line;
		ifstream myfile("exe/ConnectingDetails.txt");

		if (!myfile.is_open()) {
			cout << "Unable to open file";
			return -1;
		}
		// Otherwise, opened the file.
		getline(myfile, line);
		size_t posIP = line.find(":") + 1;
		if (posIP > 0) {

			string temp = line.substr(posIP);
			char *cstr = new char[temp.length() + 1];
			strcpy(cstr, temp.c_str());
			char temp2[10];
			strcpy(temp2, cstr);
			ipAdress = temp2;
			delete[] cstr;
		}
		getline(myfile, line);
		size_t posPort = line.find(":") + 1;
		if (posPort > 0) {
			string temp = line.substr(posPort);
			portNumber = atoi(temp.c_str());
		}
		myfile.close();

		// create the remote player
		Client firstClient(ipAdress, portNumber);

		//problem here
		firstClient.connectToServer(true); // return the true

		Board b2;

		GameLogic gameLogic(b);
		RemotePlayer firstRemotePlayer(firstClient, b); // first player.

		firstRemotePlayer.setPlayerSign(firstRemotePlayer.getPlayerSign());

		GameRunner gr(b, firstRemotePlayer, firstRemotePlayer,
				firstRemotePlayer.getPlayerSign());
		b.printBoard();

		// run the game for the remote players
		try {
			gr.run(); // run the game.
		} catch (exception *e) {
			cout << "exception was caught :( \n";
		} catch(char const* msg) {
			cout << msg;
		}


}

return 0;
}

