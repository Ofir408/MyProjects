#include "gtest/gtest.h"
#include "AIPlayer.h"
#include "MiniMaxCell.h"
#include "Board.h"
#include "Cell.h"
#include <vector>
#include "Cell.h"
#include "HumanPlayer.h"

/*  Created on: Nov 28, 2017
 *  Author: Yuval Weinstein
 */

Board createBoard1ForExample() {
	Board board;
	board.enterToBoard('O', 3, 3);
	board.enterToBoard('O', 3, 4);
	board.enterToBoard('O', 3, 5);
	board.enterToBoard('O', 5, 5);
	board.enterToBoard('X', 3, 6);
	board.enterToBoard('X', 4, 3);
	board.enterToBoard('X', 4, 4);
	board.enterToBoard('X', 4, 5);
	board.enterToBoard('X', 5, 4);

	return board;
}

Board createBoard2ForExample() {
	Board board;
	board.enterToBoard('O', 2, 2);
	board.enterToBoard('O', 3, 3);
	board.enterToBoard('O', 4, 3);
	board.enterToBoard('O', 4, 4);
	board.enterToBoard('X', 3, 6);
	board.enterToBoard('X', 4, 5);
	board.enterToBoard('X', 4, 6);
	board.enterToBoard('X', 5, 4);
	board.enterToBoard('X', 5, 5);
	board.enterToBoard('X', 5, 6);
	board.enterToBoard('X', 6, 5);

	return board;
}


TEST(GameLogicTest, PossibleMoves1) {
	Board b = createBoard1ForExample();
	vector<Cell> v;
	vector<Cell> vFunc = b.possibleCellsToAssign('O');
	int i;

	v.push_back(Cell(4, 8));
	v.push_back(Cell(6, 3));
	v.push_back(Cell(6, 4));
	v.push_back(Cell(6, 7));
	v.push_back(Cell(7, 5));

	for(i = 0; i < v.size(); i++) {
		EXPECT_TRUE(std::find(v.begin(), v.end(), vFunc.at(i)) != v.end());
	}

}

TEST(GameLogicTest, PossibleMoves2) {
	Board b = createBoard2ForExample();
	vector<Cell> v;
	vector<Cell> vFunc = b.possibleCellsToAssign('O');
	int i;

	v.push_back(Cell(5, 8));
    v.push_back(Cell(7, 5));
	v.push_back(Cell(7, 7));
	v.push_back(Cell(8, 7));

	for(i = 0; i < v.size(); i++) {
		EXPECT_TRUE(std::find(v.begin(), v.end(), vFunc.at(i)) != v.end());
	}


}

TEST(GameLogicTest, PossibleMoves3) {
	Board b = createBoard1ForExample();
	vector<Cell> v;
	vector<Cell> vFunc = b.possibleCellsToAssign('X');
	int i;

	v.push_back(Cell(3, 3));
	v.push_back(Cell(4, 3));
	v.push_back(Cell(3, 4));
	v.push_back(Cell(3, 5));
	v.push_back(Cell(3, 6));
	v.push_back(Cell(3, 7));
	v.push_back(Cell(6, 7));
	v.push_back(Cell(7, 6));
	v.push_back(Cell(7, 7));


	for(i = 0; i < v.size(); i++) {
		EXPECT_TRUE(std::find(v.begin(), v.end(), vFunc.at(i)) != v.end());
	}

}

TEST(GameLogicTest, CanAssignTest1) {
	Board b = createBoard1ForExample();
	EXPECT_FALSE(b.canAssign('O', Cell(1, 1)));
	EXPECT_TRUE(b.canAssign('O', Cell(3, 7)));
	EXPECT_TRUE(b.canAssign('O', Cell(5, 3)));
	EXPECT_TRUE(b.canAssign('X', Cell(3, 2)));
	EXPECT_TRUE(b.canAssign('X', Cell(2, 4)));
	EXPECT_FALSE(b.canAssign('X', Cell(1, 7)));
}

TEST(GameLogicTest, CanAssignTest2) {
	Board b = createBoard2ForExample();
	EXPECT_FALSE(b.canAssign('O', Cell(8, 8)));
	EXPECT_FALSE(b.canAssign('O', Cell(7, 7)));
	EXPECT_TRUE(b.canAssign('O', Cell(4, 7)));
	EXPECT_TRUE(b.canAssign('X', Cell(4, 2)));
	EXPECT_FALSE(b.canAssign('X', Cell(1, 6)));
	EXPECT_FALSE(b.canAssign('X', Cell(7, 7)));
}
