/*
 * StructDef.h
 *
 *  Created on: Dec 28, 2017
 *      Author: ofir
 */

#ifndef STRUCTDEF_H_
#define STRUCTDEF_H_
#include "GameManager.h"
#include "Server.h"
#include "map"
#include "Command.h"

// define the struct that will be sent as void*
struct passedArguments {
	int firstClientSocket;
	int secondClientSocket;
	GameManager* gameManager;
	Server* server;
	bool isInGame; // true if we start the game (into the thread of join). otherwise false.
};

struct playStruct {
	vector<string> args;
	GameManager* gameManager;
};

struct argsToThreadServer {
	int firstClientSocket;
	int secondClientSocket;
	int serverSocket;
	Server* server;
	GameManager* gameManager;
	bool* shouldContinue;
};

struct argsToExcuteCommand {
	string command;
	vector<string> args;
	GameManager* gameManager;
	map<string, Command *> commandsMap;
};

#endif /* STRUCTDEF_H_ */
