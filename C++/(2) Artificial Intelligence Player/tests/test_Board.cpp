#include "Board.h"
#include "gtest/gtest.h"
/*  Created on: Nov 27, 2017
 *  Author: Yuval Weinstein
 */
Board createBoard1ForExample();
Board createBoard2ForExample();

Board createBoard3ForExample() {
	Board board;
	int t;
	for (int i = 1; i <= 8; i++) {
		for (int j = 1; j <= 8; j++) {
			t = i*i + j*i;
			t = t%2;
			if (t == 0) {
				board.enterToBoard('O', i, j);
			} else {
				board.enterToBoard('X', i, j);
			}
		}
	}
	return board;
}

TEST(BoardTest, FullTest) {
	Board b1 = createBoard1ForExample();
	Board b2 = createBoard2ForExample();
	Board b3 = createBoard3ForExample();
	EXPECT_TRUE(b3.isBoardFull());
	EXPECT_FALSE(b1.isBoardFull());
	EXPECT_FALSE(b2.isBoardFull());
}

TEST(BoardTest, HowMuchTest) {
	Board b1 = createBoard1ForExample();
	Board b2 = createBoard2ForExample();
	Board b3 = createBoard3ForExample();

	EXPECT_EQ(b1.howMuchCells('X'), 5);
	EXPECT_EQ(b1.howMuchCells('O'), 4);
	EXPECT_EQ(b2.howMuchCells('X'), 7);
	EXPECT_EQ(b2.howMuchCells('O'), 4);
	EXPECT_EQ(b3.howMuchCells('X'), 16);
	EXPECT_EQ(b3.howMuchCells('O'), 48);

}

TEST(BoardTest, GetCharTest) {
	Board b1 = createBoard1ForExample();
	Board b2 = createBoard2ForExample();
	Board b3 = createBoard3ForExample();
	EXPECT_TRUE(b1.isCellEmpty(Cell(1,1)));
	EXPECT_EQ(b1.getPlace(2, 5), 'X');
	EXPECT_EQ(b2.getPlace(3, 3), 'O');
	EXPECT_EQ(b3.getPlace(6, 3), 'X');
}
