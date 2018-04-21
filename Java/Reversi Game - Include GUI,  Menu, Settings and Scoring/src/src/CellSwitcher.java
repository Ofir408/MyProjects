
public class CellSwitcher {
	/*
	 * CellsSwitcher
	 *
	 * Created on: Nov 5, 2017 Author: Ofir Ben-Shoham.
	 */

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
	static void rowSwitch(Board b, Cell start, Cell end, char replaceTo) {
		// start.getY() should be smaller than end.getY().
		if (start.getX() == end.getX()) {
			for (int i = start.getY(); i <= end.getY(); i++) {
				b.setCell(start.getX(), i, replaceTo);
			}
		}
	}
	
	/**
	 * As the row comment, the same input & output.
	 * However, here it's for a coulmn.
	 */
	static void colSwitch(Board b, Cell start, Cell end, char replaceTo) {
		// need to be sure that start.getX() smaller than end.getX()
		if (start.getY() == end.getY()) {
			for (int i = start.getX(); i <= end.getX(); i++) {
				b.setCell(i, start.getY(), replaceTo);
			}
		}
	}

	/**
	 * As the row comment, the same input & output.
	 * However, here it's for a slant.
	 */
	static void slantSwitch(Board b, Cell start, Cell end,
			char replaceTo) {

		if (start.getX() > end.getX() && start.getY() < end.getY()) {
			for (int row = start.getX(), col = start.getY();
					row >= end.getX() && col <= end.getY(); row--, col++) { // check if col <= end.getY() or without =
				b.setCell(row, col, replaceTo);
			}
		} else if (start.getX() > end.getX() && start.getY() > end.getY()) {
			for (int r = end.getX(), c = end.getY();
					r <= start.getX() && c <= start.getY(); r++, c++) {
				b.setCell(r, c, replaceTo);
			}
		} else if (start.getX() < end.getX() && start.getY() < end.getY()) {
			for (int row = start.getX(), col = start.getY();
					row <= end.getX() && col <= end.getY(); row++, col++) {
				b.setCell(row, col, replaceTo);
			}
		} else if (start.getX() < end.getX() && start.getY() > end.getY()) {
			for (int r = end.getX(), c = end.getY();
					r >= start.getX() && c <= start.getX(); r--, c++) {
				b.setCell(r, c, replaceTo);
			}
		}
	}
}
