
import java.util.Vector;

public class Board {

	/*
	 * Board
	 *
	 *  Created on: Oct 25, 2017
	 *      Author: Ofir Ben-Shoham.
	 */
private char[][] boardArray;

	/**
	 * Constructor function to Board, that initializes him according his
	 * start status.
	 */
	Board() {
       this.boardArray = new char[GameData.RowNumber][GameData.ColNumber];
		confirmInitialize();
		int middleRows = GameData.RowNumber/2; 
		enterToBoard('O', middleRows, middleRows);
		enterToBoard('O', middleRows+1, middleRows+1);
		enterToBoard('X', middleRows, middleRows+1);
		enterToBoard('X', middleRows+1, middleRows);
	}


	/**
	 * rowPlace - integer number that represents a place of a row.
	 * colPlace - integer number that represents a place of a column.
	 * return the current cell in the board at [rowPlace, colPlace].
	 */
	Cell getCell(int rowPlace, int colPlace)  {
		char d = this.boardArray[rowPlace][colPlace];
		if (d != 'X' && d != 'x' && d != 'O' && d != 'o') {
			d = ' ';
		}
		// +1 because the constructor of Cell with +1
		return new Cell(rowPlace + 1, colPlace + 1, d);
	}

	/**
	 * This function gets:
	 * char current player -> X / O.
	 * Cell emptyCellToCheck -> an empty cell that we check.
	 * returns true if the player can assign his sign in the input empty cell.
	 * Otherwise returns false.
	 */
	boolean canToAssign(char player, Cell emptyCellToCheck) {
		try {
			if (!isCellEmpty(emptyCellToCheck)) {
				return false; // not empty-> can't assign there.
			}
			return rowSequenceCheck(player, emptyCellToCheck, false)
					|| colSequenceCheck(player, emptyCellToCheck, false)
					|| slantSequenceCheck(player, emptyCellToCheck, false);
		} catch(Exception e ) {
			System.out.println("Into canToAssign ");
			return false; 
		}
		
	}


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
	boolean rowSequenceCheck(char player, Cell emptyCellToCheck,
			boolean wantToChange) {
		try {
			char secondPlayer = Cell.returnOtherSign(player);

			// checks first to the right side of the input Cell.
			int rightNext = emptyCellToCheck.getY() + 1;
			if (rightNext < GameData.ColNumber
					&& (isCellEmpty(emptyCellToCheck) || wantToChange)
					&& this.boardArray[emptyCellToCheck.getX()][rightNext]
							== secondPlayer) {
				// has at least one, checks more and player's sign at least one in the end ot the Sequence.
				for (int i = rightNext + 1; i <  GameData.ColNumber; i++) {
					char tempNext = boardArray[emptyCellToCheck.getX()][i];
					if (tempNext == secondPlayer) {
						continue;
					} else if (tempNext == player) {
						if (wantToChange) {
							CellSwitcher.rowSwitch(this, emptyCellToCheck,
									new Cell(emptyCellToCheck.getX() + 1, i + 1, ' '), player);
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
					&& this.boardArray[emptyCellToCheck.getX()][leftNext]
							== secondPlayer) {
				for (int i = leftNext - 1; i >= 0; i--) {
					char tempLeftNext = boardArray[emptyCellToCheck.getX()][i];
					if (tempLeftNext == secondPlayer) {
						continue;
					} else if (tempLeftNext == player) {
						if (wantToChange) {
							CellSwitcher.rowSwitch(this,
									new Cell(emptyCellToCheck.getX() + 1, i + 1, ' '),
									emptyCellToCheck, player);
						}
						return true; // has a row sequence.
					} else {
						break;
					}
				}
			}
		} catch (Exception e ) {
			System.out.println("Into ROWS check !! ");
		}
		
		// also no left sequence and also no right. therefore not found.
		return false;
	}

	/**
	 * as rowSequenceCheck algorithem input/output, just here checks coulmn and not a row.
	 * bool wantToChange - true if we want to change the board according
	 * the sequence if found.
	 * Cell's borders input are (0-7, 0-7).
	 * Note: array from 0..7 and not from 1..8
	 *
	 */
	boolean colSequenceCheck(char player, Cell emptyCellToCheck,
			boolean wantToChange) {
		try {
			char secondPlayer = Cell.returnOtherSign(player);
			// first checks upper from our input cell.
			int upperNext = emptyCellToCheck.getX() - 1;

			if (upperNext >= 0 && (isCellEmpty(emptyCellToCheck) || wantToChange)
					&& this.boardArray[upperNext][emptyCellToCheck.getY()]
							== secondPlayer) {
				for (int i = upperNext - 1; i >= 0; i--) {
					char tempUpperNext = boardArray[i][emptyCellToCheck.getY()];
					if (tempUpperNext == secondPlayer) {
						continue;
					} else if (tempUpperNext == player) {
						if (wantToChange) {
							//CellsSwitcher::colSwitch(this, Cell(i+1,emptyCellToCheck.getY() +1 ), emptyCellToCheck, player);
							for (int h = i; h <= emptyCellToCheck.getX(); h++) {
								this.setCell(h, emptyCellToCheck.getY(), player);
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
			if (belowNext <  GameData.RowNumber && upperNext <  GameData.RowNumber && (isCellEmpty(emptyCellToCheck) || wantToChange)
					&& this.boardArray[belowNext][emptyCellToCheck.getY()]
							== secondPlayer) {
				for (int i = belowNext + 1; i < GameData.RowNumber; i++) {
					char tempBelowNext = boardArray[i][emptyCellToCheck.getY()];
					if (tempBelowNext == secondPlayer) {
						continue;
					} else if (tempBelowNext == player) {
						if (wantToChange) {
							//CellsSwitcher::colSwitch(this,  emptyCellToCheck, Cell(i+1,emptyCellToCheck.getY() +1 ),player);
							for (int h = emptyCellToCheck.getX(); h <= i; h++) {
								this.setCell(h, emptyCellToCheck.getY(), player);
							}

						}

						return true; // has a row sequence.
					} else {
						break;
					}
				}
			}		
		} catch (Exception e) {
			System.out.println("Into COL Check");
		}
	
		// also no upper sequence and also no below. therefore not found.
		return false;
	}
	
	/**
	 * Gets:
	 * int row -> row number in the board. Between 0..7
	 * int col -> coulmn number in the board. Between 0..7
	 * char c -> set board[row][col] to this char.
	 */
	public void setCell(int row, int col, char c) {
		boardArray[row][col] = c;
	}

	/**
	 * The same input as slantSequenceCheck.
	 * returns true if we have a right slant from emptyCellToCheck.
	 * bool wantToChange - true if we want to change the board according
	 * the sequence if found.
	 *
	 */
	boolean rightSlantSequenceCheck(char player, Cell emptyCellToCheck,
			boolean wantToChange) {
		try {
			char secondPlayer = Cell.returnOtherSign(player);
			// upper slant.
			int r = emptyCellToCheck.getX() - 1;
			int c = emptyCellToCheck.getY() + 1;
			if (r >= 0 && c < GameData.ColNumber
					&& (isCellEmpty(emptyCellToCheck) || wantToChange)
					&& this.boardArray[r][c] == secondPlayer) {
				for (int row = r - 1, col = c + 1; row >= 0 && col < GameData.ColNumber;
						row--, col++) {
					char tempUpperNext = boardArray[row][col];
					if (tempUpperNext == secondPlayer) {
						continue;
					} else if (tempUpperNext == player) {
						if (wantToChange) {
							CellSwitcher.slantSwitch(this, emptyCellToCheck,
									new Cell(row + 1, col + 1, ' ' ), player);
						}
						return true; // has a row sequence.
					} else {
						break;
					}
				}
			}

			// below slant.
			int r2 = emptyCellToCheck.getX() + 1, c2 = emptyCellToCheck.getY() - 1;
			if (r2 < GameData.RowNumber && c2 >= 0
					&& (isCellEmpty(emptyCellToCheck) || wantToChange)
					&& this.boardArray[r2][c2] == secondPlayer) {
				for (int row2 = r2 + 1, col2 = c2 - 1; row2 < GameData.RowNumber && col2 >= 0;
						row2++, col2--) {
					char tempUpperNext = boardArray[row2][col2];
					if (tempUpperNext == secondPlayer) {
						continue;
					} else if (tempUpperNext == player) {
						if (wantToChange) {
							// CHECK IF NEED TO RETURN THIS LINE:
							CellSwitcher.slantSwitch(this, new Cell(row2 + 1, col2 + 1, ' '),
									emptyCellToCheck, player);
						}
						return true; // has a row sequence.
					} else {
						break;
					}
				}
			}
		} catch (Exception e) {
			System.out.println("Into rightSlantSequenceCheck ");
		}
	
		// if not found a sequence, return false.
		return false;
	}

	/**
	 * true of any slant has a sequnce (also right slant & left slant).
	 */
	boolean slantSequenceCheck(char player, Cell emptyCellToCheck,
			boolean wantToChange) {
		boolean tempOne = rightSlantSequenceCheck(player, emptyCellToCheck,
				wantToChange);
		boolean tempTwo = leftSlantSequenceCheck(player, emptyCellToCheck,
				wantToChange);
		return tempOne || tempTwo;
		// This needed because if the right returns true it doesn't switch in the left slant,
		// Although maybe it's also possible.
	}

	/**
	 * The same input as slantSequenceCheck.
	 * returns true if we have a left slant from emptyCellToCheck.
	 * bool wantToChange - true if we want to change the board according
	 * the sequence if found.
	 *
	 */
	boolean leftSlantSequenceCheck(char player, Cell emptyCellToCheck,
			boolean wantToChange) {
		try {
			char secondPlayer = Cell.returnOtherSign(player);
			// below left slant.
			int r = emptyCellToCheck.getX() + 1, c = emptyCellToCheck.getY() + 1;
			if (r < GameData.RowNumber && c < GameData.ColNumber
					&& (isCellEmpty(emptyCellToCheck) || wantToChange)
					&& this.boardArray[r][c] == secondPlayer) {
				for (int row = r + 1, col = c + 1; row <  GameData.RowNumber && col < GameData.ColNumber;
						row++, col++) {
					char tempBelowNext = boardArray[row][col];
					if (tempBelowNext == secondPlayer) {
						continue;
					} else if (tempBelowNext == player) {
						if (wantToChange) {
							CellSwitcher.slantSwitch(this, emptyCellToCheck,
									new Cell(row + 1, col + 1, ' ' ), player);
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
			if (r < 0)
				r++; 
			if (c < 0)
				c++; 
			if (r < GameData.RowNumber && c < GameData.ColNumber
					&& (isCellEmpty(emptyCellToCheck) || wantToChange)
					&& this.boardArray[r][c] == secondPlayer) {
				for (int row = r - 1, col = c - 1; row >= 0 && col >= 0; row--, col--) {
					char tempBelowNext = boardArray[row][col];
					if (tempBelowNext == secondPlayer) {
						continue;
					} else if (tempBelowNext == player) {
						if (wantToChange) {
							// change for slant // CellsSwitcher::rowSwitch(this, Cell(emptyCellToCheck.getX()+1, i+1), emptyCellToCheck, player);
							CellSwitcher.slantSwitch(this, new Cell(row + 1, col + 1, ' '),
									emptyCellToCheck, player);
						}
						return true; // has a row sequence.
					} else {
						break;
					}
				}
			}
		} catch (Exception e ) {
			System.out.println("Into LEFT SLANT CHECK \n");
		}
		
		// if not found a sequence, return false.
		return false;
	}

	void enterToBoard(char signToAdd, int row, int col) {
		if (row >= 1 && row <= 20) {
			this.boardArray[row - 1][col - 1] = signToAdd;
		} else {
			System.out.println( " PROBLEM IN Board::enterToBoard ");
			System.exit(1);
		}
	}

	/**
	 * returns true if this cell is empty by players signs..
	 */
	boolean isCellEmpty(Cell cell)  {
		char current = this.boardArray[cell.getX()][cell.getY()];
		if (cell.getX() < 0 || cell.getY() < 0 || cell.getX() >= GameData.RowNumber || cell.getY() >= GameData.ColNumber )
			return false; 
		if (current != 'X' && current != 'x' && current != 'o' && current != 'O') {
			return true;
		}
		return false;
	}

	/**
	 * 
	 * returns true if the board is full.
	 */
	boolean  isBoardFull() {
		for (int r = 0; r < GameData.RowNumber; r++) {
			for (int c = 0; c < GameData.ColNumber; c++) {
				char current = boardArray[r][c];
				if (current != 'X' && current != 'O' && current != 'x'
						&& current != 'o') {
					return false; // does not  full
				}
			}
		}
		return true;
	}
	
	/**
	 * returns vector of cells that each one represents an empty cell that
	 * the current player can assign his sign there.
	 * each cell from the return vector is from (1..8, 1..8)
	 * Because we want to print the possible moves
	 * to the current player according how the board is printed and not from (0..7) as the array works.
	 *
	 */
	Vector<Cell> possibleCellsToAssign(char player) {
		Vector<Cell> vecToReturn = new Vector<Cell>();
		// passing on the all cells in our board.
		for (int r = 1; r <= GameData.RowNumber; r++) {
			for (int c = 1; c <= GameData.ColNumber; c++) {
				Cell currentCell = new Cell(r, c, ' ');
				if (isCellEmpty(currentCell) && canToAssign(player, currentCell)) {
					// add it to our vector with +1 because we want to show to our user
					// the possible cells from (1..8, 1..8) and not from (0..7, 0..7).
					vecToReturn.addElement(
							new Cell(currentCell.getX() + 2, currentCell.getY() + 2, ' ')); // +2 because we lower twice when we created Cell using its constructor.
				}
			}
		}
		return vecToReturn;
	}
	
	/**
	 * ensure that all the values in our board initilized to space.
	 */
	void confirmInitialize() {
		for (int r = 0; r < GameData.RowNumber; r++) {
			for (int c = 0; c < GameData.ColNumber; c++) {
				char dummy = ' ';
				this.boardArray[r][c] = dummy;
			}
		}
}
	
	/**
	 * return the length of the board.
	 */
	int getLength() {
		return this.boardArray.length;
	}
	
	/**
	 * return the width of the board.
	 */
	int getWidth() {
		return this.boardArray[0].length;
	}
	
	/**
	 * returns how much pawns the input player has into our board.
	 */
	int getPlayerNumberOfPawns(char player) {
		int count = 0;
		for (int i = 0; i < this.boardArray.length; i++) {
			for (int j = 0; j < this.boardArray[0].length; j++) {
				if (this.boardArray[i][j] == player) {
					count++;
				}
			}
		}
		return count;
	}
}