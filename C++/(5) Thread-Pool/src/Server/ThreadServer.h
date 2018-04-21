/*
 * ThreadServer.h
 *
 *  Created on: Dec 28, 2017
 *      Author: Ofir Ben-Shoham.
 */

#ifndef THREADSERVER_H_
#define THREADSERVER_H_
#include "Server.h"
#include "GameManager.h"

namespace std {

class ThreadServer {
public:
	/**
	 * c'tor for ThreadServer
	 */
	ThreadServer(Server* server);

	/**
	 * d'tor for ThreadServer
	 */
	~ThreadServer();

	/**
	 * open with a new thread, runs a loop to look for exit command in server.
	 */
	static void* gamesRunner(void* input);

	/**
	 * runs the server's loop to look for new client connections
	 */
	void runServerLoop();

	/**
	 * exits from all the games in the server
	 * gameMng is the gameManager of the server to close the games from
	 * server is our server
	 */
	static void exitFromAllTheGames(GameManager* gameMng, Server* server);

	/**
	 * returns the pointer to the boolean var that determains if the server loop should continue
	 */
	bool* getPointerToShouldContinue();
private:
	bool* shouldContinue;
	Server* server;
};

} /* namespace std */

#endif /* THREADSERVER_H_ */
