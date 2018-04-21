/*
 * Board.h
 *
 *  Created on: Oct 25, 2017
 *      Author: Ofir Ben-Shoham.
 *      Id    : 208642496.
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <iostream>
#include "Cell.h"
#include "vector"

using namespace std;

class Board {
public:

	/**
	 * Constructor function to Board, that initializes him according his
	 * start status.
	 */
	Board();

	/**
	 * void function.
	 * printBoard prints the current status of the board.
	 */
	void printBoard() const;

	/**
	 * rowPlace - integer number that represents a place of a row.
	 * colPlace - integer number that represents a place of a column.
	 * return a char in the board at [rowPlace, colPlace].
	 */
	char getPlace(int rowPlace, int colPlace) const;

	/**
	 * rowPlace - integer number that represents a place of a row.
	 * colPlace - integer number that represents a place of a column.
	 * return the current cell in the board at [rowPlace, colPlace].
	 */
	Cell getCell(int rowPlace, int colPlace) const;

	/**
	 * rowPlace - integer number that represents a place of a row.
	 * colPlace - integer number that represents a place of a column.
	 * return a string after convert the char at
	 * board[rowPlace, colPlace] to a string.
	 */
	string helperGetPlace(int rowPlace, int colPlace) const;

	/**
	 * checks who won in the game, it means - which player
	 * has more cells. This method is called when the game
	 * is ended.
	 */
	void whoWon() const;

	/**
	 * Gets:
	 * char signToAdd -> char to add to the board at the givven place.
	 * row -> row to add into from 1...8 and not from 0...7, what the function does is
	 * to take row 1 for example and initialzies it in row 0 at our array, because it
	 * starts from 0 to 7 and not until 8 as our printed board.
	 * col -> col to add into from 1...8, the same comment as row.
	 * To summerize, for example if gets ('X', 8, 8) put in (7,7)-'x'.
	 */
	void enterToBoard(char signToAdd, int row, int col);

	/**
	 * helper method that gets a Cell in the board, and a player,
	 * then checks if there is sequence of the other player, in order to check
	 * if the input player can to assign his sign in the input cell.
	 * The method returns true if the input player can to put in the
	 * input empty cell.
	 * For example: Cell(1,4), player X:   O O X -> return yes.
	 * Cell's borders input are (0-7, 0-7).
	 * --- Note: array from 0..7 and not from 1..8 ---
	 */
	bool rowSequenceCheck(char player, Cell emptyCellToCheck,
			bool wantToChange = false);

	/**
	 * as rowSequenceCheck algorithem input/output, just here checks coulmn and not a row.
	 * bool wantToChange - true if we want to change the board according
	 * the sequence if found.
	 * Cell's borders input are (0-7, 0-7).
	 * Note: array from 0..7 and not from 1..8
	 *
	 */
	bool colSequenceCheck(char player, Cell emptyCellToCheck,
			bool wantToChange = false);

	/**
	 * slant checking, the same algorithem as above.
	 * bool wantToChange - true if we want to change the board according
	 * the sequence if found.
	 * Cell's borders input are (0-7, 0-7).
	 * array from 0..7 and not from 1..8
	 */
	bool slantSequenceCheck(char player, Cell emptyCellToCheck,
			bool wantToChange = false);

	/**
	 * The same input as slantSequenceCheck.
	 * returns true if we have a right slant from emptyCellToCheck.
	 * bool wantToChange - true if we want to change the board according
	 * the sequence if found.
	 *
	 */
	bool rightSlantSequenceCheck(char player, Cell emptyCellToCheck,
			bool whatToChange = false);

	/**
	 * The same input as slantSequenceCheck.
	 * returns true if we have a left slant from emptyCellToCheck.
	 * bool wantToChange - true if we want to change the board according
	 * the sequence if found.
	 *
	 */
	bool leftSlantSequenceCheck(char player, Cell emptyCellToCheck,
			bool whatToChange = false);

	/**
	 * This function gets:
	 * char current player -> X / O.
	 * Cell emptyCellToCheck -> an empty cell that we check.
	 * returns true if the player can assign his sign in the input empty cell.
	 * Otherwise returns false.
	 */
	bool canToAssign(char player, Cell emptyCellToCheck);

	/**
	 * Cell cell - the cell that we want to check if it's empty.
	 * check if cell is empty, if yes returns true. Otherwise false.
	 */
	bool isCellEmpty(Cell cell);

	/**
	 * Gets:
	 * int row -> row number in the board. Between 0..7
	 * int col -> coulmn number in the board. Between 0..7
	 * char c -> set board[row][col] to this char.
	 */
	void setCell(int row, int col, char c) {
		boardArray[row][col] = c;
	}

	/**
	 * returns true if the board is full.
	 * Otherwise returns false.
	 */
	bool isBoardFull();

	/**
	 * Gets:
	 * char playerToCheck - the player that we want to check how much cells
	 * he has on the board.
	 * For example, if gets x counting how much sings at the board have the
	 * sign x.
	 * Returns an integer number that represents the number of cells the input
	 * player has on the board.
	 *
	 */
	int howMuchCells(char playerToCheck);

	/**
	 * copy constructor for board, in order do not change him
	 * while we check the mini-max algorithem.
	 */
	Board copyConstructor(Board toCopy);

	/**
	 * returns vector of cells that each one represents an empty cell that
	 * the current player can assign his sign there.
	 * each cell from the return vector is from (1..8, 1..8)
	 * Because we want to print the possible moves
	 * to the current player according how the board is printed and not from (0..7) as the array works.
	 *
	 */
	vector<Cell> possibleCellsToAssign(char player);

	/**
	 * char player -> X\O, the sign of the current player.
	 * Cell input  -> the input that the player wrote,
	 * here we assume that his choose
	 * is valid.
	 */
	void inputAssignManager(char player, Cell input);

	/**
	 * This method checks if the input player can to assign the input cell
	 * in our board.
	 * if it's possible return true. Otherwise -> false.
	 * Cell cellToCheck borders are from: (0..7, 0..7) after the constructor.
	 */
	bool canAssign(char player, Cell cellToCheck);

private:
	// array that represents our board.
	char boardArray[RowNumber][colNumber];

	/**
	 * void function.
	 * rowNumber - integer number that represents the row that
	 * we want to print from our board.
	 * as her name, this method gets a row and prints her.
	 */
	void printOneLine(int rowNumbers) const;

	/**
	 * helper method that gets:
	 * char player, char secondPlayer - who is the current player and who is the second.
	 * char cellSign what the sign in the cell that we check into.
	 */
	bool helperChecker(char player, char secondPlayer, char cellSign) const;

	/**
	 * ensure that all the values in our board initilized to space.
	 */
	void confirmInitialize();

};

#endif /* BOARD_H_ */
