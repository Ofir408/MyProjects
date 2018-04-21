/*
 * GameDescriptor.h
 *
 *  Created on: Dec 22, 2017
 *      Author: Ofir Ben-Shoham
 */

#ifndef GAMEDESCRIPTOR_H_
#define GAMEDESCRIPTOR_H_
#include <string>

using namespace std;

class GameDescriptor {
public:

	GameDescriptor(string name, int firstSoc, int secondSoc) { nameOfGame = name; firstClientSocket =  firstSoc;
	secondClientSocket = secondSoc;}

	/* get the int of the sokcet of the first client */
	int getFirstClientSocket() {return firstClientSocket;}

	/* get the name of this Game */
	string& getNameOfGame() {return nameOfGame;}

	/* get the int of the sokcet of the second client */
	int getSecondClientSocket() {return secondClientSocket;}

	/**
	 * returns true if this game is with just the first player,
	 * and no has the second player.
	 * It means, if we have just the first client socket and not the second.
	 */
	bool hasJustOneClient() { return firstClientSocket > 0 && secondClientSocket <= 0; }

	void setFirstClientSocket(int fClientSocket) {
		this->firstClientSocket = fClientSocket;
	}

	void setSecondClientSocket(int sClientSocket) {
		this->secondClientSocket = sClientSocket;
	}

	/**
	 * returns true if the game is empty, it means that two players doesn't connect.
	 * If both connects, return false.
	 */
	bool isGameWithoutPlayers() { return firstClientSocket <= 0 && secondClientSocket <= 0; }

	/**
	 * return true if two players connects to the game.
	 * Otherwise, false.
	 */
	bool IsBothPlayersConnects() { return firstClientSocket > 0 && secondClientSocket >0; }

private:

	/**
	 * the game is composed from two clients sockets that the server connects
	 * between them.
	 */
	int firstClientSocket, secondClientSocket;

	/**
	 * each game has its name.
	 */
	string nameOfGame;
};


#endif /* GAMEDESCRIPTOR_H_ */
