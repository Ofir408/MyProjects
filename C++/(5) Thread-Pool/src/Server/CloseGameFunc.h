/*
 * CloseGameFunc.h
 *
 *  Created on: Dec 23, 2017
 *      Author: Ofir Ben-Shoham.
 */

#ifndef CLOSEGAMEFUNC_H_
#define CLOSEGAMEFUNC_H_
#include "Command.h"

using namespace std;

class CloseGameFunc: public Command {
public:
	CloseGameFunc();

	/**
	 * virtual method that need to be implemented because it's a derive class of Command
	 * Gets vector<string> ->
	 * Into args[2] : the name of the game to close.
	 * It's the programmer responssiblity to give him this, when calling excute with list_games command.
	 *
	 */
	virtual void execute(vector<string> args, GameManager* gameManager);
};

#endif /* CLOSEGAMEFUNC_H_ */
