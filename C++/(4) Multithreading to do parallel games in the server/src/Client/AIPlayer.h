/*
 * AIPlayer.h
 * The AI player extends Player "interface", because it has
 * it's own implementation of the makeMove method.
 *  Created on: Nov 23, 2017
 *      Author: ofir
 */

#ifndef AIPLAYER_H_
#define AIPLAYER_H_

#include "Player.h"
#include "Board.h"
#include "MiniMaxCell.h"
#include "GameLogic.h"
//#include <vector>

namespace std {

class AIPlayer: public Player {
public:

	/**
	 * Remark of this function at Player.h
	 */
	Cell chooseCell(Board *board, char playerSign);

	char getPlayerSign() { return this->playerSign; }


	/**
	 * constructor method of AI player.
	 * Gets:
	 * char pSign - the sign of the player.
	 */
	AIPlayer(char Psign) {
		Player::setPlayerSign(Psign); // very important
		playerSign = Psign;
	}

	/**
	 * returns the sign of the AI player.
	 */
	char getAIplayerSign() {
		return playerSign;
	}

	/**
	 * Part 2B in the mini-max pasudo code.
	 * Gets a vector of cells, that represents the possible cells
	 * Return MiniMaxCell object that contains minimum grade of the cell and the cell itself.
	 * This function takes care to parts 2b and 2c in the pesudo code.
	 * Gets:
	 * vector<Cell> possibleCells - all the possible cells of the enemy in the
	 * new board status (to make part 2b).
	 * Return the maximum grade the the enemy can to get in the new board status.
	 */

	static MiniMaxCell getEnemyCurrentMaxGrade(Board &board,
			vector<Cell> possibleCells, char enemySign);

	/**
	 * Gets board and enemySign, then:
	 *  put the possible cell in the board. Board doesn't change because this
	 *  method gets (&) of the fake board (that changed) and not a pointer to our real board.
	 *  now will check what is the grade of the enemy and returns this grade.
	 */
	static int getEnemyCurrentGrade(Board &board, char enemySign);

	/**
	 * Gets board and player sign and returns his next move to play,
	 * a MiniMaxCell object with the data of the grade and in which cell.
	 */
	static MiniMaxCell returnNextMove(Board &board, char playerSign);

private:
	char playerSign; // the sign of the AI player in the board.
};

} /* namespace std */

#endif /* AIPLAYER_H_ */

