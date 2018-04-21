/*
 * PlayTurnFunc.h
 *
 *  Created on: Dec 23, 2017
 *      Author: ofir
 */

#ifndef PLAYTURNFUNC_H_
#define PLAYTURNFUNC_H_
#include "Command.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include "StructDef.h"

namespace std {

class PlayTurnFunc : public Command {
public:
	PlayTurnFunc();

	/**
	 * virtual method that need to be implemented because it's a derive class of Command
	 * Gets vector<string> ->
	 * Into args[0] : the client that sent his turn.
	 * Into args[1] : the other client that the server will send him the turn.
	 * Into args[2] : x value of the cell that the client choose.
	 * Into args[3] : y value of the cell that the client choose.
	 * It's the programmer responssiblity to give him this, when calling excute with list_games command.
	 *
	 */
	virtual void execute(vector<string> args, GameManager* gameManager);
	static void* threadPlayExcute (void* playSt);
};

} /* namespace std */

#endif /* PLAYTURNFUNC_H_ */
