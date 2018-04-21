/*
 * GameManager.h
 *
 *  Created on: Dec 22, 2017
 *      Author: Ofir Ben-Shoham.
 */

#ifndef GAMEMANAGER_H_
#define GAMEMANAGER_H_
#include "GameDescriptor.h"
#include "vector"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <pthread.h>
#include <map>

using namespace std;

class GameManager {
public:
	GameManager();

	/**
	 * Open new game - nameOfNewGame
	 * will add he game with the input string to our games vector.
	 * Moreover, will add this thread in order to handle with the current new game.
	 */
	void addGame(string nameOfNewGame, int firstSocketNum, int secondSocketNum);

	/**
	 * Remove Game - nameOfNewGame
	 * will remove the game from the games vector.
	 * Moreover, will kill the relevant thread from execution.
	 */
	void removeGame(string nameOfGameToRemove);

	/**
	 * search into gamesDesList the game with the name "nameToFind".
	 * return his index. If doesn't found return -1.
	 */
	int findGameAccordingString(string nameToFind);

	map<string, pthread_t> getThreadsMap();

	/**
	 * search into gamesDesList the game with the name "nameToFind".
	 * return the game if was found, otherwise return NULL.
	 */
	GameDescriptor* getGameAccordingString(string nameToFind);

	/**
	 * return string with the names of the game.
	 * for example if we have into gamesDesList: firstGame, secondGame, third.. etc
	 * will return the string: firstGame secondGame third (space seperate between them).
	 */
	string getsGameNames();


	/**
	 * the same as getsGameNames() but returns just the names of the game
	 * with just * ONE * player.
	 */
	string getsOnePlayerGameNames();

	void setThread(pthread_t thread, string forGame);
	void exitThread(string forGame);

	vector<GameDescriptor*> getGamesDesList() {
		return gamesDesList;
	}

private:
	// member fileds.

	/**
	 * vector of all the games, each one has its own game data.
	 */
	vector<GameDescriptor*> gamesDesList;

	/**
	 * array of threads to be made, each one can be started when using
	 * "join" command for him.
	 */
	map<string, pthread_t> threads;
};

#endif /* GAMEMANAGER_H_ */
