/*
 * ThreadStarter.h
 *
 *  Created on: Dec 27, 2017
 *      Author: ofir
 */

#ifndef THREADSTARTER_H_
#define THREADSTARTER_H_

#include "GameManager.h"
#include "CommandsManager.h"
#include "StartGameFunc.h"
#include "ListGamesFunc.h"
#include "JoinGameFunc.h"
#include "PlayTurnFunc.h"
#include "CloseGameFunc.h"
#include "StructDef.h"
#include <sstream>

namespace std {

class ThreadStarter {


public:
	/**
	 * static function that gets void* - we will pass the struct with the data.
	 * And then send and write between the clients.
	 */
	static void* workWithClients(void* des);

	/**
	 * initialize the map and call to the command with the arguments as a vector<string>,
	 * using in getArgs function.
	 */
	static void* readAndRunCommand(void* des);

	/**
	 * gets the arguments passed by the command from client.
	 * command is the string the client sent
	 * return a vector of the commands
	 */
	static vector<string> getArgs(string command);

	/**
	 * converts an int to a string
	 */
	static string convertIntToString(int i);

	/**
	 * gets the command itself from the string the client sent
	 * command is the string the client sent
	 * return the command
	 */
	static string getCommand(string command);
};

} /* namespace std */

#endif /* THREADSTARTER_H_ */
