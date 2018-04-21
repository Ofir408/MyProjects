/*
 * ThreadStarter.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: Ofir Ben-Shoham.
 */

#include "ThreadStarter.h"
#include <algorithm>

using namespace std;

void* ThreadStarter::workWithClients(void* des) {

	struct passedArguments *gameDes = (struct passedArguments *) des;
	struct passedArguments gameDescription; //gameDescription to save
	gameDescription.firstClientSocket = gameDes->firstClientSocket;
	gameDescription.secondClientSocket = gameDes->secondClientSocket;
	gameDescription.gameManager = gameDes->gameManager;
	delete gameDes; //free the memory allocated

	int firstClientSocket = gameDescription.firstClientSocket;
	int secondClientSocket = gameDescription.secondClientSocket;

	if (firstClientSocket <= 0 || secondClientSocket <= 0) {
		cout
				<< "I don't have the sockets of the clients, can't continue. ThreadStarter::workWithClients() \n";

		exit(-1);
	}

	// Connect them before the loop because we want to send "1 to player 1 & 2 to player 2
	// Just once and not always.

	// start the while loop until END was sent.

	while (true) {
		try {
			ThreadStarter::readAndRunCommand(&gameDescription);
					int temp;
					temp = gameDescription.firstClientSocket;
					gameDescription.firstClientSocket = gameDescription.secondClientSocket;
					gameDescription.secondClientSocket = temp;
		} catch (char const* msg) {
			pthread_exit(NULL); // exit from the current game thread.
		}
	}

	// when it comes to here, it means that we exited from the while and the game is ended.
}

void* ThreadStarter::readAndRunCommand(void* des) {
	char buffer[100];
	passedArguments *gameDes = (passedArguments*) des; // convert from void*
	int sendingClientSock = gameDes->firstClientSocket;
	int receivingClientSock = gameDes->secondClientSocket;
	GameManager* gameManager = gameDes->gameManager;

	//reading command to execute from client
	int n = read(sendingClientSock, buffer, sizeof(buffer));
	if (n <= 0) {
		throw "could not read in expectStartCommand method";
	}
	//creating an empty arguments vector to return at the end
	vector<string> vec;

	//getting the arguments sent by the sending Client
	vector<string> args = ThreadStarter::getArgs(buffer);

	//pushing the sockets to the vector as first arguments (they are needed as well)
	vec.push_back(convertIntToString(sendingClientSock));
	vec.push_back(convertIntToString(receivingClientSock));
	unsigned int i = 1;
	for (i = 1; i < args.size(); i++) {
		// add the data except the first arg that it is a command (as play, close, join.. etc)
		vec.push_back(args[i]);
	}

	// build the command manager
	map<string, Command *> commandsMap;

	commandsMap["start"] = new StartGameFunc();
	commandsMap["list_games"] = new ListGamesFunc();
	commandsMap["join"] = new JoinGameFunc(ThreadStarter::workWithClients);
	commandsMap["play"] = new PlayTurnFunc();
	commandsMap["close"] = new CloseGameFunc();

	// build the command manager and return it
	CommandsManager cmToReturn(commandsMap);

	cmToReturn.executeCommand(args[0], vec, gameManager);

}

string ThreadStarter::getCommand(string command) {
	return command.substr(0, command.find('<'));
}

vector<string> ThreadStarter::getArgs(string command) {
	vector<string> vec = vector<string>(); //will contain the parameters of the command
	unsigned int i = 0, howManyArg, tempIndex = 0, space,
			lastIndexOfTempVar = 0;
	string whatToDo;
	howManyArg = count(command.begin(), command.end(), '<');

	// first getting the command (player/ join anc etc..)
	if (howManyArg <= 1) {
		if (command.find("list_games") <= command.length()) {
			vec.push_back("list_games");
			return vec;
		}
	}
	space = command.find(' ');
	whatToDo = command.substr(0, space);
	vec.push_back(whatToDo);

	string tempString = command.substr(space, command.length() - 1);
	while (i < howManyArg) {
		//as long as there are parameters
		tempIndex = tempString.find('<');
		lastIndexOfTempVar = tempString.find('>') + 1;
		string SecondtempString = tempString.substr(tempIndex,
				lastIndexOfTempVar);

		string toInsert = SecondtempString.substr(
				SecondtempString.find('<') + 1, SecondtempString.find('>') - 1);
		vec.push_back(toInsert);
		tempString = tempString.substr(tempIndex + 3, tempString.length());
		i++;
	}

	return vec;
}

string ThreadStarter::convertIntToString(int i) {
	ostringstream temp;
	temp << i;
	return temp.str();
}

