/*
 * ListGamesFunc.h
 *
 * This class takes care to the second option that the client can send to the server.
 * The command : list_games
 *  Created on: Dec 22, 2017
 *      Author: Ofir Ben-Shoham.
 */

#ifndef LISTGAMESFUNC_H_
#define LISTGAMESFUNC_H_
#include "Command.h"
#include <unistd.h>

namespace std {

class ListGamesFunc : public Command {
public:
	ListGamesFunc();

	/**
	 * virtual method that need to be implemented because it's a derive class of Command
	 * Gets vector<string> -> for this class the vector should contain in args[0]
	 *  **the client socket number** that asked the return lists.
	 * It's the programmer responssiblity to give him this, when calling excute with list_games command.
	 *
	 */
	virtual void execute(vector<string> args, GameManager* gameManager);
};

} /* namespace std */

#endif /* LISTGAMESFUNC_H_ */
