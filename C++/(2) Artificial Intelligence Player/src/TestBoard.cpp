/*
 * TestBoard.cpp
 *
 *  Created on: Nov 27, 2017
 *      Author: ofir
 */

#include "Board.h"

void testSequenceCheck() {
	Board b;
	if (b.rowSequenceCheck('O', Cell(3, 3)) != false) {
		cout << "(1) failed";
	}
	if (b.rowSequenceCheck('O', Cell(1, 3)) != false) {
		cout << "(2) failed";
	}
	if (b.colSequenceCheck('O', Cell(7, 3)) != false) {
		cout << "(3) failed";
	}
	if (b.rowSequenceCheck('X', Cell(4, 3)) != true) {
		cout << "(4) failed";
	}

	if (b.colSequenceCheck('X', Cell(3, 4)) != true) {
		cout << "(5) failed";
	}

	if (!b.canToAssign('X', Cell(3, 4))) {
		cout << "(6) failed";
	}
	if (!b.canToAssign('O', Cell(3, 4))) {
		cout << "(7) failed";
	}

	if (b.isCellEmpty(Cell(1, 1))) {
		cout << "(8) failed";
	}
	if (!b.isCellEmpty(Cell(4, 4))) {
		cout << "(9) failed";
	}

	if (b.isBoardFull()) {
		cout << "(10) failed";
	}

	if (b.possibleCellsToAssign('X').empty()) {
		cout << "(11) failed";
	}

	if (b.canAssign('X', Cell(4,4))) {
		cout << "(12) failed";
	}

}

