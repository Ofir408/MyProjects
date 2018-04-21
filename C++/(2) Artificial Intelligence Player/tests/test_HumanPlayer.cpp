#include "gtest/gtest.h"
#include "HumanPlayer.h"
#include "Board.h"

/*  Created on: Nov 27, 2017
 *  Author: Yuval Weinstein
 */

Board createBoard2ForExample();

TEST(HumanPlayerTests, ValidInputTest) {
	Board b1;
	Board b2 = createBoard2ForExample();
	HumanPlayer p('X');
	HumanPlayer p2('O');

	b2.printBoard();
	EXPECT_TRUE(p.checkValidInput(b1, Cell(3, 4)));
	EXPECT_FALSE(p.checkValidInput(b1, Cell(1, 1)));
	EXPECT_TRUE(p.checkValidInput(b2, Cell(1, 1)));
	EXPECT_FALSE(p.checkValidInput(b2, Cell(8, 1)));
	EXPECT_TRUE(p2.checkValidInput(b2, Cell(4, 7)));
	EXPECT_FALSE(p.checkValidInput(b2, Cell(7, 7)));
	EXPECT_TRUE(p2.checkValidInput(b2, Cell(6, 6)));
	EXPECT_FALSE(p2.checkValidInput(b2, Cell(4, 2)));
}
