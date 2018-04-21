/*
 * Board.cpp
 *
 *  Created on: Oct 25, 2017
 *      Author: Ofir Ben-Shoham.
 *      Id    : 208642496.
 *      Note -< all the methods remarks at the Board header file.
 */

#include "Board.h"
#include "Cell.h"
#include <string.h>
#include <sstream>
#include "CellsSwitcher.h"
#include "vector"

/**
 * Constructor function to Board, that initializes him according his
 * start status.
 */
Board::Board() {

	confirmInitialize();
	enterToBoard('O', 4, 4);
	enterToBoard('O', 5, 5);
	enterToBoard('X', 4, 5);
	enterToBoard('X', 5, 4);
}

void Board::printBoard() const {

	cout << "The current status of the board is: " << endl << endl;
	for (int i = 1; i <= 8; i++) {
		cout << " | " << i;
	}
	cout << " | " << endl;

	for (int i = 0; i < RowNumber; i += 1) {
		this->printOneLine(i);
	}
	cout << "----------------------------------" << endl;
}

void Board::printOneLine(int rowNumber) const {
	cout << "----------------------------------" << endl;
	cout << (rowNumber + 1) << "|";
	cout << " " << helperGetPlace(rowNumber, 0) << " | "
			<< helperGetPlace(rowNumber, 1) << " | "
			<< helperGetPlace(rowNumber, 2) << " | "
			<< helperGetPlace(rowNumber, 3) << " | "
			<< helperGetPlace(rowNumber, 4) << " | "
			<< helperGetPlace(rowNumber, 5) << " | "
			<< helperGetPlace(rowNumber, 6) << " | ";
	cout << helperGetPlace(rowNumber, 7) << " |" << endl;
}

char Board::getPlace(int rowPlace, int colPlace) const {

	char d = this->boardArray[rowPlace][colPlace];
	if (d != 'X' && d != 'O') {
		return ' ';
	}
	return this->boardArray[rowPlace][colPlace];
}

string Board::helperGetPlace(int rowPlace, int colPlace) const {
	stringstream temp;
	string s;
	char current = this->getPlace(rowPlace, colPlace);
	if (current == ' ') {
		return " ";
	}

	temp << current; // in order to move from char to a string
	temp >> s;
	return s;
}

Cell Board::getCell(int rowPlace, int colPlace) const {
	char d = this->boardArray[rowPlace][colPlace];
	if (d != 'X' && d != 'x' && d != 'O' && d != 'o') {
		d = ' ';
	}
	// +1 because the constructor of Cell with +1
	return Cell(rowPlace + 1, colPlace + 1, d);
}

bool Board::canToAssign(char player, Cell emptyCellToCheck) {
	if (!isCellEmpty(emptyCellToCheck)) {
		return false; // not empty-> can't assign there.
	}
	return rowSequenceCheck(player, emptyCellToCheck)
			|| colSequenceCheck(player, emptyCellToCheck)
			|| slantSequenceCheck(player, emptyCellToCheck);
}

bool Board::helperChecker(char player, char secondPlayer, char cellSign) const {
	if (cellSign == player) {
		return true; // has a row sequence.
	}
	return false; // not X and not O -> empty cell and no sequence.
}

bool Board::rowSequenceCheck(char player, Cell emptyCellToCheck,
		bool wantToChange) {
	char secondPlayer = Cell::returnOtherSign(player);

	// checks first to the right side of the input Cell.
	int rightNext = emptyCellToCheck.getY() + 1;
	if (rightNext <= colNumber
			&& (isCellEmpty(emptyCellToCheck) || wantToChange)
			&& this->boardArray[emptyCellToCheck.getX()][rightNext]
					== secondPlayer) {
		// has at least one, checks more and player's sign at least one in the end ot the Sequence.
		for (int i = rightNext + 1; i < colNumber; i++) {
			char tempNext = boardArray[emptyCellToCheck.getX()][i];
			if (tempNext == secondPlayer) {
				continue;
			} else if (tempNext == player) {
				if (wantToChange) {
					CellsSwitcher::rowSwitch(this, emptyCellToCheck,
							Cell(emptyCellToCheck.getX() + 1, i + 1), player);
				}
				return true; // has a row sequence.
			} else {
				break;
			}
		}
	}
	// now checks to the left side of the input Cell.

	int leftNext = emptyCellToCheck.getY() - 1;
	if (leftNext >= 0 && (isCellEmpty(emptyCellToCheck) || wantToChange)
			&& this->boardArray[emptyCellToCheck.getX()][leftNext]
					== secondPlayer) {
		for (int i = leftNext - 1; i >= 0; i--) {
			char tempLeftNext = boardArray[emptyCellToCheck.getX()][i];
			if (tempLeftNext == secondPlayer) {
				continue;
			} else if (tempLeftNext == player) {
				if (wantToChange) {
					CellsSwitcher::rowSwitch(this,
							Cell(emptyCellToCheck.getX() + 1, i + 1),
							emptyCellToCheck, player);
				}
				return true; // has a row sequence.
			} else {
				break;
			}
		}
	}
	// also no left sequence and also no right. therefore not found.
	return false;
}

bool Board::colSequenceCheck(char player, Cell emptyCellToCheck,
		bool wantToChange) {
	char secondPlayer = Cell::returnOtherSign(player);
	// first checks upper from our input cell.
	int upperNext = emptyCellToCheck.getX() - 1;

	if (upperNext >= 0 && (isCellEmpty(emptyCellToCheck) || wantToChange)
			&& this->boardArray[upperNext][emptyCellToCheck.getY()]
					== secondPlayer) {
		for (int i = upperNext - 1; i >= 0; i--) {
			char tempUpperNext = boardArray[i][emptyCellToCheck.getY()];
			if (tempUpperNext == secondPlayer) {
				continue;
			} else if (tempUpperNext == player) {
				if (wantToChange) {
					for (int h = i; h <= emptyCellToCheck.getX(); h++) {
						this->setCell(h, emptyCellToCheck.getY(), player);
					}
				}
				return true; // has a row sequence.
			} else {
				break;
			}
		}
	}

	// now checks below our input cell
	int belowNext = emptyCellToCheck.getX() + 1;
	if (upperNext < RowNumber && (isCellEmpty(emptyCellToCheck) || wantToChange)
			&& this->boardArray[belowNext][emptyCellToCheck.getY()]
					== secondPlayer) {
		for (int i = belowNext + 1; i < RowNumber; i++) {
			char tempBelowNext = boardArray[i][emptyCellToCheck.getY()];
			if (tempBelowNext == secondPlayer) {
				continue;
			} else if (tempBelowNext == player) {
				if (wantToChange) {
					//CellsSwitcher::colSwitch(this,  emptyCellToCheck, Cell(i+1,emptyCellToCheck.getY() +1 ),player);
					for (int h = emptyCellToCheck.getX(); h <= i; h++) {
						this->setCell(h, emptyCellToCheck.getY(), player);
					}

				}
				//cout << "BB wantToChange is:  " << wantToChange;
				return true; // has a row sequence.
			} else {
				break;
			}
		}
	}
	// also no upper sequence and also no below. therefore not found.
	return false;
}

bool Board::rightSlantSequenceCheck(char player, Cell emptyCellToCheck,
		bool wantToChange) {
	char secondPlayer = Cell::returnOtherSign(player);
	// upper slant.
	int r = emptyCellToCheck.getX() - 1;
	int c = emptyCellToCheck.getY() + 1;
	if (r >= 0 && c < colNumber
			&& (isCellEmpty(emptyCellToCheck) || wantToChange)
			&& this->boardArray[r][c] == secondPlayer) {
		for (int row = r - 1, col = c + 1; row >= 0 && col < colNumber;
				row--, col++) {
			char tempUpperNext = boardArray[row][col];
			if (tempUpperNext == secondPlayer) {
				continue;
			} else if (tempUpperNext == player) {
				if (wantToChange) {
					CellsSwitcher::slantSwitch(this, emptyCellToCheck,
							Cell(row + 1, col + 1), player);
				}
				return true; // has a row sequence.
			} else {
				break;
			}
		}
	}

	// below slant.
	int r2 = emptyCellToCheck.getX() + 1, c2 = emptyCellToCheck.getY() - 1;
	if (r2 < RowNumber && c2 >= 0
			&& (isCellEmpty(emptyCellToCheck) || wantToChange)
			&& this->boardArray[r2][c2] == secondPlayer) {
		for (int row2 = r2 + 1, col2 = c2 - 1; row2 < RowNumber && col2 >= 0;
				row2++, col2--) {
			char tempUpperNext = boardArray[row2][col2];
			if (tempUpperNext == secondPlayer) {
				continue;
			} else if (tempUpperNext == player) {
				if (wantToChange) {
					// CHECK IF NEED TO RETURN THIS LINE:
					CellsSwitcher::slantSwitch(this, Cell(row2 + 1, col2 + 1),
							emptyCellToCheck, player);
				}
				return true; // has a row sequence.
			} else {
				break;
			}
		}
	}
	// if not found a sequence, return false.
	return false;
}

bool Board::slantSequenceCheck(char player, Cell emptyCellToCheck,
		bool wantToChange) {
	bool tempOne = rightSlantSequenceCheck(player, emptyCellToCheck,
			wantToChange);
	bool tempTwo = leftSlantSequenceCheck(player, emptyCellToCheck,
			wantToChange);
	return tempOne || tempTwo;
	// This needed because if the right returns true it doesn't switch in the left slant,
	// althought maybe it's also possible.
}

bool Board::leftSlantSequenceCheck(char player, Cell emptyCellToCheck,
		bool wantToChange) {
	char secondPlayer = Cell::returnOtherSign(player);
	// below left slant.
	int r = emptyCellToCheck.getX() + 1, c = emptyCellToCheck.getY() + 1;
	if (r < RowNumber && c < colNumber
			&& (isCellEmpty(emptyCellToCheck) || wantToChange)
			&& this->boardArray[r][c] == secondPlayer) {
		for (int row = r + 1, col = c + 1; row < RowNumber && col < colNumber;
				row++, col++) {
			char tempBelowNext = boardArray[row][col];
			if (tempBelowNext == secondPlayer) {
				continue;
			} else if (tempBelowNext == player) {
				if (wantToChange) {
					CellsSwitcher::slantSwitch(this, emptyCellToCheck,
							Cell(row + 1, col + 1), player);
				}
				return true; // has a row sequence.
			} else {
				break;
			}
		}
	}

	// upper left slant
	r = emptyCellToCheck.getX() - 1;
	c = emptyCellToCheck.getY() - 1;
	if (r < RowNumber && c < colNumber
			&& (isCellEmpty(emptyCellToCheck) || wantToChange)
			&& this->boardArray[r][c] == secondPlayer) {
		for (int row = r - 1, col = c - 1; row >= 0 && col >= 0; row--, col--) {
			char tempBelowNext = boardArray[row][col];
			if (tempBelowNext == secondPlayer) {
				continue;
			} else if (tempBelowNext == player) {
				if (wantToChange) {
					CellsSwitcher::slantSwitch(this, Cell(row + 1, col + 1),
							emptyCellToCheck, player);
				}
				return true; // has a row sequence.
			} else {
				break;
			}
		}
	}
	// if not found a sequence, return false.
	return false;
}

void Board::enterToBoard(char signToAdd, int row, int col) {
	if (row >= 1 && row <= 8) {
		this->boardArray[row - 1][col - 1] = signToAdd;
	} else {
		cout << " PROBLEM IN Board::enterToBoard \n ";
		exit(1);
	}
}

bool Board::isCellEmpty(Cell cell) {
	char current = this->boardArray[cell.getX()][cell.getY()];
	if (current != 'X' && current != 'x' && current != 'o' && current != 'O') {
		return true;
	}
	return false;
}

void Board::whoWon() const {
	int xCounter = 0, oCounter = 0;
	for (int r = 0; r < RowNumber; r++) {
		for (int c = 0; c < colNumber; c++) {
			if (this->boardArray[r][c] == 'X')
				xCounter += 1;
			if (this->boardArray[r][c] == 'O')
				oCounter += 1;
		}
	}
	if (xCounter > oCounter) {
		cout << "Player X Won !! " << endl;
		cout << "Player X: " << xCounter << ", Player O: " << oCounter << endl;

	} else if (oCounter > xCounter) {
		cout << "Player O Won !! " << endl;
		cout << "Player O: " << oCounter << ", Player X: " << xCounter << endl;

	} else {
		cout << "DRAW" << endl;
		cout << "Player X: " << xCounter << ", Player O: " << oCounter << endl;
	}
}

bool Board::isBoardFull() {
	for (int r = 0; r < RowNumber; r++) {
		for (int c = 0; c < colNumber; c++) {
			char current = boardArray[r][c];
			if (current != 'X' && current != 'O' && current != 'x'
					&& current != 'o') {
				return false; // doesnt full
			}
		}
	}
	return true;
}

void Board::confirmInitialize() {
	for (int r = 0; r < RowNumber; r++) {
		for (int c = 0; c < colNumber; c++) {
			char dummy = ' ';
			this->boardArray[r][c] = dummy;
		}
	}
}

int Board::howMuchCells(char playerToCheck) {
	int counter = 0;
	for (int r = 0; r < RowNumber; r++) {
		for (int c = 0; c < colNumber; c++) {
			if (this->boardArray[r][c] == playerToCheck) {
				counter += 1;
			}
		}
	}
	return counter;
}

vector<Cell> Board::possibleCellsToAssign(char player) {
	vector<Cell> vecToReturn;
	// passing on the all cells in our board.
	for (int r = 1; r <= RowNumber; r++) {
		for (int c = 1; c <= colNumber; c++) {
			Cell currentCell(r, c);
			if (isCellEmpty(currentCell) && canToAssign(player, currentCell)) {
				// add it to our vector with +1 because we want to show to our user
				// the possible cells from (1..8, 1..8) and not from (0..7, 0..7).
				vecToReturn.push_back(
						Cell(currentCell.getX() + 2, currentCell.getY() + 2)); // +2 because we lower twice when we created Cell using its constructor.
			}
		}
	}
	return vecToReturn;
}

Board Board::copyConstructor(Board toCopy) {
	Board b;
	b.confirmInitialize();
	for (int i = 0; i < RowNumber; i++) {
		for (int c = 0; c < colNumber; c++) {
			b.boardArray[i][c] = toCopy.boardArray[i][c];
		}
	}
	return b;
}

void Board::inputAssignManager(char player, Cell input) {
	if (this->canAssign(player, input)) {
		this->rowSequenceCheck(player, input, true);
		this->colSequenceCheck(player, input, true);
		this->slantSequenceCheck(player, input, true);
	}
}

bool Board::canAssign(char player, Cell cellToCheck) {
	vector<Cell> possibleMovesVec = this->possibleCellsToAssign(player);

	for (unsigned i = 0; i < possibleMovesVec.size(); i++) {
		Cell currentCell = Cell(possibleMovesVec[i].getX(),
				possibleMovesVec[i].getY());
		if (currentCell.compareCells(cellToCheck)) {
			return true;
		}
	}
	return false;
}

void Board::printPossibleCells(char player) {
	vector<Cell> possibleMovesVec = this->possibleCellsToAssign(player);
	cout << "Player: " << player << " ,Your Possible Moves are: ";
	for (unsigned i = 0; i < possibleMovesVec.size(); i++) {
		possibleMovesVec[i].printCell();
	}
	cout
			<< " \nPlease choose a cell from your possible options, for example write 3 (Enter) 4 (Enter):"
			<< endl;
}

bool Board::canContinue(char c) {
	return !this->possibleCellsToAssign(c).empty();
}
