public class Cell {
	/*
	 * Cell.cpp
	 *
	 * Created on: Oct 31, 2017 Author: Ofir Ben-Shoham.
	 */
	private int x, y;
	private char sign;

	/**
	 * constructor method.
	 * Gets board places x,y and a char (X, O) to place in.
	 */
	Cell(int newX, int newY, char newSign) {
		x = newX - 1;
		y = newY - 1;
		sign = newSign;

		if (newSign != 'O' && newSign != 'o' && newSign != 'X' && newSign != 'x') {
			sign = ' ';
		}
	}

	/**
	 * returns x index of this cell in the board.
	 */
	public int getX() {
		return x;
	}

	/**
	 * val to set to.
	 */
	public void setX(int x) {
		this.x = x;
	}

	/**
	 * returns y index of this cell in the board.
	 */
	public int getY() {
		return y;
	}

	/**
	 * set y
	 */
	public void setY(int y) {
		this.y = y;
	}

	/**
	 * return the sign of this cell.
 	 */
	public char getSign() {
		return sign;
	}

	/**
	 *  set the sign of this cell.
 	 */
	public void setSign(char sign) {
		this.sign = sign;
	}

	/**
	 * returns true if this cell is valid.
	 */
	boolean isValid() {
		return (x >= 0 && x < GameData.RowNumber && y >= 0 && y < GameData.ColNumber);
	}
	
	/**
	 * void function.
	 * Gets nothing, just help us to print the current cell.
	 */
	void printCell() {
		System.out.println( " (" + getX() + ", " + getY() + ")");
	}
	
	/**
	 * Gets:
	 * Cell other - the other cell that we want to compare with.
	 * returns true if the cells equal. Otherwise returns false.
	 */
	boolean compareCells(Cell other) {
		return x == other.getX() && y == other.getY();
	}


	/**
	 * get other sign. For x return 0 and for 0 return X.
	 */
	static char returnOtherSign(char currentSign) {
		if (currentSign == 'X' || currentSign == 'x') {
			return 'O';
		}
		if (currentSign == 'O' || currentSign == 'o') {
			return 'X';
		}
		return ' ';
	}
	/**
	 * returns true if in this cell we have X
	 */
	boolean hasX() {
		boolean b = this.sign == 'X' || this.sign == 'x';
		return b;
	}


	/**
	 * returns true if in this cell we have O
	 */
	boolean hasO() {
		boolean b = this.sign == 'O' || this.sign == 'o';
		return b;
	}

	/**
	 * returns true this cell is empty. Otherwise -> false.
	 */
	boolean isEmptyCell() {
		return !hasO() && !hasX();
	}

}
