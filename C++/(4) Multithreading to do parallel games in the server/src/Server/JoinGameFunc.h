/*
 * JoinGameFunc.h
 *
 *  Created on: Dec 23, 2017
 *      Author: Ofir Ben-Shoham
 */

#ifndef JOINGAMEFUNC_H_
#define JOINGAMEFUNC_H_
#include "Command.h"
#include <unistd.h>

using namespace std;

class JoinGameFunc: public Command {
public:

	// define the struct that will be sent as void*
	struct passedArguments {
		int firstClientSocket;
		int secondClientSocket;
		GameManager* gameManager;
	};

	JoinGameFunc(void* (*funcToStart)(void*));

	/**
	 * virtual method that need to be implemented because it's a derive class of Command
	 * Gets vector<string> -> for this class the vector should contain in args[0]
	 *  **the SECOND client socket number** that asked to enter the game.
	 *  In args[2] - the name of the game that the second client want to enter.
	 *  To sum up: args[0]- socket of the client that wants to enter to args[2] game (name of game).
	 * It's the programmer responssiblity to give him this, when calling excute with list_games command.
	 *
	 */
	virtual void execute(vector<string> args, GameManager* gameManager);

private:

	/**
	 * pointer to the function that each thread needs to start with
	 */
	void* (*funcToStart)(void*);
};

#endif /* JOINGAMEFUNC_H_ */
