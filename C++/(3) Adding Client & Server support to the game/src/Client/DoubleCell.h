/*
 * DoubleCell.h
 *
 *  Created on: Dec 8, 2017
 *      Author: ofir
 */

#ifndef TEMP_DOUBLECELL_H_
#define TEMP_DOUBLECELL_H_
#include "Cell.h"
namespace std {

class DoubleCell {
public:
	DoubleCell(Cell otherMove, Cell currentMove ) { otherPlayerMove = otherMove; currentPlayerMove = currentMove; }
	DoubleCell(){}

	const Cell& getCurrentPlayerMove() const {
		return currentPlayerMove;
	}

	const Cell& getOtherPlayerMove() const {
		return otherPlayerMove;
	}

	void setCurrentPlayerMove(Cell currentPlayerMove) {
		this->currentPlayerMove = currentPlayerMove;
	}

	void setOtherPlayerMove(Cell otherPlayerMove) {
		this->otherPlayerMove = otherPlayerMove;
	}

private:
	Cell otherPlayerMove, currentPlayerMove;
};

} /* namespace std */

#endif /* TEMP_DOUBLECELL_H_ */
