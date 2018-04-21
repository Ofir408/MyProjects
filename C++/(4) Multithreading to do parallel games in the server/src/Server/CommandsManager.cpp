/*
 * CommandsManager.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: Ofir Ben-Shoham.
 */

#include "CommandsManager.h"
#include "StructDef.h"
#include "PlayTurnFunc.h"

using namespace std;

CommandsManager::CommandsManager(map<string, Command *> newCommandsMap) {
	this->commandsMap = newCommandsMap;
}

CommandsManager::~CommandsManager() {
	map<string, Command *>::iterator it;
	for (it = commandsMap.begin(); it != commandsMap.end(); it++) {
		delete it->second;
	}
	// need to write the deletion of gamesList

	/*for (unsigned int i = 0; i < this->gamesList->size(); i++) {
	 GameDescriptor current = gamesList[i];
	 delete this->gamesList[i];
	 }
	 delete gamesList;*/
}

void CommandsManager::executeCommand(string command, vector<string> args,
		GameManager* gameManager) {
        if (command == "play") {
		struct playStruct* p = new playStruct;
		p->args = args;
		pthread_t thread;
		// add the needed in the next line:
	PlayTurnFunc::threadPlayExcute((void*)p);
		/*pthread_create(&thread, NULL, PlayTurnFunc::threadPlayExcute,
				(void*) p);*/
		return ; // works good.
	}
	Command *commandObj = commandsMap[command];
	commandObj->execute(args, gameManager);
}
