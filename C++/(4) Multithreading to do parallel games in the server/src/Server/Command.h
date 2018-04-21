/*
 * Command.h
 *
 *  Created on: Dec 22, 2017
 *      Author: Ofir Ben-Shoham.
 */

#ifndef COMMAND_H_
#define COMMAND_H_

#include <vector>
#include <string>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include "GameManager.h"

using namespace std;
class Command {

public:
 virtual void execute(vector<string> args, GameManager* gameManager) = 0; // to be implemente in the derived classes.
 virtual ~Command() {}
};



#endif /* COMMAND_H_ */
