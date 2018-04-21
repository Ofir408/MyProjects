/*
 * StartGameFunc.h
 * This class takes care to the First option that the client can send to the server.
 * The command : start <name> that start a game with the input name.
 *  Created on: Dec 22, 2017
 *      Author: Ofir Ben-Shoham
 */

#ifndef STARTGAMEFUNC_H_
#define STARTGAMEFUNC_H_
#include "Command.h"
using namespace std;

class StartGameFunc: public Command {
public:

	StartGameFunc();

	/**
	 * virtual method that need to be implemented because it's a derive class of Command
	 * Gets vector<string> -> for this class the vector should contain in args[0]
	 *  **the first client socket number** that asks to open new game. Moreover,
	 * via args[2] the name of the new game should be.
	 * To Sum Up: args[0] - socket number of first client, args[2] - string, the name of the game.
	 * It's the programmer responssiblity to give him this, when calling excute with list_games command.
	 *
	 */
	virtual void execute(vector<string> args, GameManager* gameManager);
};


#endif /* STARTGAMEFUNC_H_ */
