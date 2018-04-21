/*
 * HumanPlayer.h
 *
 *  Created on: Nov 24, 2017
 *     Names : Yuval Weinstein & Ofir Ben Shoham.
 *     Id: 208580613 & 208642496.
 */

#ifndef HUMANPLAYER_H_
#define HUMANPLAYER_H_

#include "Player.h"

namespace std {

class HumanPlayer: public Player {
public:

	/**
	 * constructor function of HumanPlayer.
	 * Gets:
	 * char  newPlayerSign - X or O, the char of the human player
	 * on the board.
	 */
	HumanPlayer(char newPlayerSign) {
		Player::setPlayerSign (newPlayerSign); // very important
		playerSign = newPlayerSign;
	}

	Cell chooseCell(Board *board, char playerSign);


	char getPlayerSign() { return this->playerSign; }


	/**
	 * function that ask for input from the user.
	 * returns the Cell that it gets from the user after cheking it's a valid one.
	 */
	Cell askInput();

	/**
	 * takes care to get the input from the user, check if legal.
	 * If it is, return this input cell.
	 * Otherwise, continue to ask for input until gets a legal one.
	 */
	Cell getUserInput(Board &b);

	/**
	 * print possible moves in the board for an input player.
	 */
	void printPossibleMoves(Board &b);

	/**
	 * Gets :
	 * Cell inputCell - the input cell that the user choose.
	 * Board &b - to check if this cell in board's borders.
	 * Returns true if it's possible to put in our board this Cell.
	 * Otherwise, return false.
	 */
	bool checkValidInput(Board &b, Cell inputCell);

private:
	char playerSign;
};

} /* namespace std */

#endif /* HUMANPLAYER_H_ */
