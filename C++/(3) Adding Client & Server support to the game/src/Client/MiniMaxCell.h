/*
 * MiniMaxCell.h
 * This class just save data.
 * Has: Cell in the board.
        And the grade of this move if we put the cell in the board.
 *  Created on: Nov 23, 2017
 *      Author: ofir
 */

#ifndef MINIMAXCELL_H_
#define MINIMAXCELL_H_

#include "Cell.h"
namespace std {

class MiniMaxCell {
public:

	MiniMaxCell(){gradeForThisCell = 1000;}
	MiniMaxCell(Cell cell, int gradeForInputCell);

	/**
	 * return the cell
	 */
	Cell& getCellToPut() {
		return cellToPut;
	}

	/**
	 * return the grade for this cell.
	 */
	int getGradeForThisCell() const {
		return gradeForThisCell;
	}

	/**
	 * Replace miniMaxCell with input other miniMaxCell
	 */
	void replace(Cell cell, int gradeForInputCell) {
		this->cellToPut = Cell(cell.getX() + 1, cell.getY() + 1);
		this->gradeForThisCell = gradeForInputCell;
	}

	/**
	 * Replace with the same type object but diffrent one.
	 */
	void replace(MiniMaxCell other) {
		this->replace(other.getCellToPut(), other.getGradeForThisCell());
	}

private:
	Cell cellToPut;
	int gradeForThisCell;
};

} /* namespace std */

#endif /* MINIMAXCELL_H_ */
