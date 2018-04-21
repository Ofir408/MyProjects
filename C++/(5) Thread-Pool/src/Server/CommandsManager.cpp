/*
 * CommandsManager.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: Ofir Ben-Shoham.
 */

#include "CommandsManager.h"
#include "PlayTurnFunc.h"

using namespace std;

/*CommandsManager::CommandsManager(map<string, Command *> newCommandsMap) {
 this->commandsMap = newCommandsMap;
 }

 CommandsManager::~CommandsManager() {
 map<string, Command *>::iterator it;
 for (it = commandsMap.begin(); it != commandsMap.end(); it++) {
 delete it->second;
 }*/
// need to write the deletion of gamesList
/*for (unsigned int i = 0; i < this->gamesList->size(); i++) {
 GameDescriptor current = gamesList[i];
 delete this->gamesList[i];
 }
 delete gamesList;*/
//}
void* CommandsManager::executeCommand(void* dataToExcute) {

	struct argsToExcuteCommand* argsToEx = (argsToExcuteCommand*) dataToExcute;
	vector<string> args = argsToEx->args;
	GameManager* gameManager = argsToEx->gameManager;
	string command = argsToEx->command;
	map<string, Command *> commandsMap = argsToEx->commandsMap;

	if (command == "play") {
		struct playStruct* p = new playStruct;
		p->args = args;
		pthread_t thread;
		// add the needed in the next line:
		pthread_create(&thread, NULL, PlayTurnFunc::threadPlayExcute,
				(void*) p);
		return (void*) 1; // works good.
	}
	Command *commandObj = commandsMap[command];
	commandObj->execute(args, gameManager);
	return (void*) 1; // works good.

}
