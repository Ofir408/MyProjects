 /*
 * GameManager.cpp
 *
 *  Created on: Dec 22, 2017
 *      Author: Ofir Ben-Shoham
 */

#include "GameManager.h"

using namespace std;

GameManager::GameManager() {
	this->gamesDesList.empty(); // no games in the start.
}


void GameManager::removeGame(string nameOfGameToRemove) {
	// find the element
	unsigned int indexToRemove = findGameAccordingString(nameOfGameToRemove);
	if (indexToRemove < 0 || indexToRemove >= this->gamesDesList.size()) {
		cout << "No such element to remove into in GameManager::removeGame\n";
		return; // exit from function, no such element to be remove.
	}
	GameDescriptor* tempToRemove =  this->gamesDesList.at(indexToRemove);
	pthread_cancel(this->threads[nameOfGameToRemove]);
	this->threads.erase(nameOfGameToRemove);
	this->gamesDesList.erase(gamesDesList.begin() + indexToRemove); // remove the element.
	delete tempToRemove;
}

void GameManager::addGame(string nameOfNewGame, int firstSocketNum,
		int secondSocketNum) {
	GameDescriptor *toAdd = new GameDescriptor(nameOfNewGame, firstSocketNum, secondSocketNum);
	this->gamesDesList.push_back(toAdd); // add him in the end of the list.g
}


int GameManager::findGameAccordingString(string nameToFind) {
	unsigned int i = 0;
	int indexToReturn = -1;
	string currentName;
	for (i = 0; i < this->gamesDesList.size(); i++) {
		currentName = this->gamesDesList[i]->getNameOfGame();
		if (currentName.compare(nameToFind) == 0) {
			indexToReturn = i;
			break; // allready found.
		}
	}
	return indexToReturn;
}


string GameManager::getsGameNames() {
	unsigned int i;
	string toReturn = "";
	for (i = 0; i < this->gamesDesList.size(); i++) {
		toReturn += this->gamesDesList[i]->getNameOfGame();
	}
	return toReturn;
}

GameDescriptor* GameManager::getGameAccordingString(string nameToFind) {
	int index = this->findGameAccordingString(nameToFind);
	if (index == -1) {
		// we don't have a game in this name.
		cout << "We don't have a game in this name. GameManager::getGameAccordingString\n";
		throw ("Can't continue GameManager::getGameAccordingString\n");
	} else {
		return this->getGamesDesList()[index]; // return pointer to the current game.
	}
}

string GameManager::getsOnePlayerGameNames() {
	unsigned int i;
	string toReturn = "";
	for (i = 0; i < this->gamesDesList.size(); i++) {
		if (this->gamesDesList[i]->hasJustOneClient()) {
			if (i > 0 && i < this->gamesDesList.size()) {
				toReturn += " ";
			}
				toReturn += this->gamesDesList[i]->getNameOfGame();
		}
	}
	return toReturn;

}

void GameManager::setThread(pthread_t thread, string forGame) {
	this->threads[forGame] = thread;
}

void GameManager::exitThread(string forGame) {
//pthread_exit(this->threads(forGame));
	this->threads.erase(forGame);
}

map<string, pthread_t> GameManager::getThreadsMap() {
	return this->threads;
}
