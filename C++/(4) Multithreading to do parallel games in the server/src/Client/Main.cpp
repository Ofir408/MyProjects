/*
 * Main.cpp
 *
 *  Created on: Dec 2, 2017
 *     Names : Yuval Weinstein & Ofir Ben Shoham.
 *     Id: 208580613 & 208642496.
 */
#include <stdio.h>
#include <string>
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
		ifstream myfile("ConnectingDetails.txt");

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

again:
		firstClient.connectToServer();
		//presenting menu to choose whether to open a new game or to join an existing game
		string option = GameRunner::startOrJoinGame();
		string gameName = "";
		if (option.compare("list_games") == 0) {
			cout << firstClient.askForGameList() << endl << endl;
			goto again;
		}
		gameName = GameRunner::inputGameName();
		int status = firstClient.makeFirstMessage(option, gameName);
		if (status == -1) {
			if (option == "start") {
				cout << "name already exists. please choose again" << endl;
			} else if (option == "join") {
				cout << "no such game exists. please choose again" << endl;
			}
			goto again;
		}
		firstClient.setTurnByCommand(option);

		Board b2;
		GameLogic gameLogic(b);
		RemotePlayer firstRemotePlayer(firstClient, b); // first player.

		firstRemotePlayer.setPlayerSign(firstRemotePlayer.getPlayerSign());

		GameRunner gr(b, firstRemotePlayer, firstRemotePlayer,
				firstRemotePlayer.getPlayerSign());

		b.printBoard();
		cout
				<< "I Know that our game looks fun, and you want to play.. But please wait to the other player to join :) \n";

		// wait to get connected message before asking for user input.
		char buffer[20];
		memset(buffer, 0, sizeof(buffer)); // reset the buffer.


		/*if (read(firstClient.getClientSocket(), buffer, sizeof(buffer)) <= 0) {
			close(firstClient.getClientSocket());
			cout << endl << endl
					<< "Dear Player, the server asks to exit from the game.\n I hope that you enjoyed to play into our game.. As a little favor,  "
					<< "Please give us a grade that will make us happy :) \n";
		}

		/*if (strcmp(buffer, "-1") == 0) {
			cout << "You chose unvalid game name, please choose other name\n";
			goto askNameAgain;
		}*/

		/*if (strcmp(buffer, "wait") == 0) {
			// read again, until play send.
			if (read(firstClient.getClientSocket(), buffer, sizeof(buffer))
					<= 0) {
				close(firstClient.getClientSocket());
				cout << endl << endl
						<< "Dear Player, the server asks to exit from the game.\n I hope that you enjoyed to play into our game.. As a little favor,  "
						<< "Please give us a grade that will make us happy :) \n";
				exit(0);
			}
		}*/

		// run the game for the remote players
		try {
			cout << "Both player are connected, let's start in the game! \n";
			gr.run(); // run the game.
		} catch (exception *e) {
			cout << "exception was caught :( \n";
		} catch (char const* msg) {
			cout << msg;
		}

	}
	return 0;
}

