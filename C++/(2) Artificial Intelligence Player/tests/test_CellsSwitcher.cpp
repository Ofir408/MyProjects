#include "gtest/gtest.h"
#include "CellsSwitcher.h"
#include "Board.h"

/*  Created on: Nov 27, 2017
 *  Author: Yuval Weinstein
 */

Board createBoard4ForExample() {
	Board b;
	b.setCell(3, 1, 'X');
	b.setCell(3, 2, 'O');
	b.setCell(3, 3, 'O');
	b.setCell(3, 4, 'O');
	b.setCell(3, 5, 'X');

	b.setCell(6, 1, 'O');
	b.setCell(5, 2, 'X');
	b.setCell(4, 3, 'X');
	b.setCell(3, 4, 'O');

	b.setCell(1, 7, 'O');
	b.setCell(2, 7, 'X');
	b.setCell(3, 7, 'X');
	b.setCell(4, 7, 'X');
	b.setCell(5, 7, 'O');

	b.setCell(3, 1, 'X');
	b.setCell(4, 2, 'O');
	b.setCell(5, 3, 'O');
	b.setCell(6, 4, 'X');

	return b;
}

TEST(CellsSwitcherTests, RowSwitchTest) {
	Board b2 = createBoard4ForExample();
	CellsSwitcher cs;

	cs.rowSwitch(&b2, Cell(3, 1), Cell(3, 5), 'X');

	for (int i = 0; i <= 4; i++) {
		EXPECT_EQ(b2.getCell(2, i).getSign(), 'X');
	}
}

TEST(CellsSwitcherTests, ColSwitchTest) {
	Board b2 = createBoard4ForExample();
	CellsSwitcher cs;

	cs.colSwitch(&b2, Cell(1, 7), Cell(5, 7), 'O');

	for (int i = 0; i <= 4; i++) {
		EXPECT_EQ(b2.getCell(i, 6).getSign(), 'O');
	}
}

TEST(CellsSwitcherTests, SlantSwitchTest) {
	Board b2 = createBoard4ForExample();
	CellsSwitcher cs;

	cs.slantSwitch(&b2, Cell(6, 1), Cell(3, 4), 'O');
	cs.slantSwitch(&b2, Cell(3, 1), Cell(6, 4), 'X');

	for (int i = 0; i <= 3; i++) {
		EXPECT_EQ(b2.getCell(5-i, i).getSign(), 'O');
	}

	for (int i = 0; i <= 3; i++) {
		EXPECT_EQ(b2.getCell(i+2, i).getSign(), 'X');
	}

}



