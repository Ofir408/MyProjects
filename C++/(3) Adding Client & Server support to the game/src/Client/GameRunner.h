/*
 * GameRunner.h
 *
 *  Created on: Nov 9, 2017
 *      Author: Ofir Ben-Shoham.
 */

#ifndef GAMERUNNER_H_
#define GAMERUNNER_H_
#include "Cell.h"
#include "Board.h"
#include "GameLogic.h"
#include "AIPlayer.h"
#include "Player.h"

namespace std {

class GameRunner {
public:

	/**
	 * Constructor of GameRunner.
	 * Gets:
	 * Board b - a board that changes in our game.
	 * char player - the player who starts the game. By defualt, it will be 'X'.
	 * The player character changes each turn.
	 */
	GameRunner(Board b, Player& firstPlayer, Player& secondPlayer, char startingPlayer = 'X');
	GameRunner(){secondPlayer = NULL; firstPlayer = NULL; currentPlayer = ' ';}


	/**
	 * This function makes the next move for the current player.
	 */
	void playNextMove(Player &playerCurrentTurn);

	/**
	 * run & manage the game, from the beginning until it ends.
	 */
	void run();

	/**
	 * return true if we can continue in the game.
	 * Otherwise -> return false.
	 */
	bool canToContinue();

	/**
	 * returns the &Player according char currentPlayer,
	 * it means, return the player who has the current turn to play.
	 */
	Player& getCurrentPlayerTurn();

	/**
	 * Takes care to show the user the game's option.
	 * Then he chooses if he wants to play vs an human player
	 * Or an AI player that works with mini-max algorithem.
	 * Return 1 if want to play vs human player
	 * Return 2 if want to play vs AI player
	 * If the input doesn't 1 or 2, ask that user the input again.
	 */
	static int menu();

	void setSecondPlayer(Player& secondPlayer) {
		this->secondPlayer = &secondPlayer;
	}

private:
	Board board;
	Player *firstPlayer, *secondPlayer;
	GameLogic gameLogic;
	char currentPlayer;

	/**
	 * switch the current player after the second made his move.
	 * It means, change the currentPlayer field into GameRunner.
	 */
	void switchCurrentPlayer();

};

} /* namespace std */

#endif /* GAMERUNNER_H_ */
