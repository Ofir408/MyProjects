/*
 * CellsSwitcher.h
 *
 *  Created on: Nov 5, 2017
 *      Author: ofir ben shoham
 *      Id    : 208642496.
 */

#ifndef CELLSSWITCHER_H_
#define CELLSSWITCHER_H_
#include "Board.h"
namespace std {

class CellsSwitcher {
public:
	CellsSwitcher();
	/**
	 * method that is called when there is a ROW sequence and we want to replace
	 *  the other player signs.
	 *  For example : X O O O X we will replace it to X X X X X.
	 *  Cells borders are here: (0..7, 0..7, as the array borders.
	 *  Gets: Board b - board to change the signs into him.
	 *  Cell start -> from where to start change.
	 *  Cell end   -> where to stop replace.
	 *  char replaceTo -> to which char replace (X or O).
	 *  For example: rowSwitch(b, (0,0), (0, 5), X):
	 *  Initial status: X 0 0 0 O X replace to :
	 *  X X X X X X
	 */
	void static rowSwitch(Board *b, Cell start, Cell end, char replaceTo);
	/**
	 * As the row comment, the same input & output.
	 * However, here it's for a coulmn.
	 */
	void static colSwitch(Board *b, Cell start, Cell end, char replaceTo);

	/**
	 * As the row comment, the same input & output.
	 * However, here it's for a slant.
	 */
	void static slantSwitch(Board *b, Cell start, Cell end, char replaceTo);
};

} /* namespace std */

#endif /* CELLSSWITCHER_H_ */
