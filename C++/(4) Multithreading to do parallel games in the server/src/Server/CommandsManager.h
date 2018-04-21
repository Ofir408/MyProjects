/*
 * CommandsManager.h
 *
 *  Created on: Dec 23, 2017
 *      Author: Ofir Ben-Shoham.
 */


using namespace std;
#include <map>
#include "Command.h"

class CommandsManager {

public:
	/**
	 * constructor.
	 */
	CommandsManager(map<string, Command *> newCommandsMap);

	/**
	 * distructor, free the memory.
	 */
	~CommandsManager();

	/**
	 * this function using in command design pattern. Gets:
	 * string command - the command to make, that the client send to the server.
	 * vector<string> args - all the rellevant arguments to make the input command.
	 * For example:
	 * string command: "play". And the vector<string> should be as follows:
	 * Into args[0] : the client that sent his turn.
	 * Into args[1] : the other client that the server will send him the turn.
	 * Into args[2] : x value of the cell that the client choose.
	 * Into args[3] : y value of the cell that the client choose.
	 */
	void executeCommand(string command, vector<string> args, GameManager* gameManager);

private:
	map<string, Command *> commandsMap;
};

