#include "gtest/gtest.h"
#include "AIPlayer.h"
#include "MiniMaxCell.h"
#include "Board.h"
#include "Cell.h"
#include <vector>
#include "Cell.h"

/*  Created on: Nov 23, 2017
 *  Author: Yuval Weinstein
 */
Board createBoard1ForExample();
Board createBoard2ForExample();

TEST(AITest, AIRightMove1) {
	AIPlayer p('O');
	Board b = createBoard1ForExample();
	vector<Cell> v;

	v.push_back(Cell(4, 8));
	v.push_back(Cell(6, 3));
	v.push_back(Cell(6, 4));
	v.push_back(Cell(6, 7));
	v.push_back(Cell(7, 5));

	MiniMaxCell mmc = p.getEnemyCurrentMaxGrade(b, v, 'X');
	int x = mmc.getCellToPut().getX();
	int y = mmc.getCellToPut().getY();
	EXPECT_EQ(x, 3);
	EXPECT_EQ(y, 7);
}

TEST(AITest, AIRightMove2) {
	AIPlayer p('O');
	Board b = createBoard2ForExample();

	Cell c = p.chooseCell(&b, 'O');

	EXPECT_EQ(c.getX(), 3);
	EXPECT_EQ(c.getY(), 6);
}

TEST(AITest, AIRightMove3) {
	AIPlayer p('O');
	Board b = createBoard1ForExample();
	vector<Cell> v;

	v.push_back(Cell(4, 8));
	v.push_back(Cell(6, 3));
	v.push_back(Cell(6, 4));
	v.push_back(Cell(6, 7));
	v.push_back(Cell(7, 5));

	MiniMaxCell mmc = p.getEnemyCurrentMaxGrade(b, v, 'X');
	int x = mmc.getCellToPut().getX();
	int y = mmc.getCellToPut().getY();
	EXPECT_EQ(x, 3);
	EXPECT_EQ(y, 7);
}

TEST(AITest, AIRIGHTGRADE1) {
	AIPlayer p('O');
	Board b = createBoard1ForExample();
	EXPECT_EQ(p.getEnemyCurrentGrade(b, 'O'), -1);
}

TEST(AITest, AIRIGHTGRADE2) {
	AIPlayer p('O');
	Board b = createBoard2ForExample();
	EXPECT_EQ(p.getEnemyCurrentGrade(b, 'O'), -3);
}

TEST(AITest, AIRightSign) {
	AIPlayer p('t');
	EXPECT_EQ(p.getAIplayerSign(), 't');
}

TEST(AITest, EnemyMiniMaxGrade1) {
	AIPlayer p('O');
	Board b = createBoard1ForExample();
	vector<Cell> v;

	v.push_back(Cell(4, 8));
	v.push_back(Cell(6, 3));
	v.push_back(Cell(6, 4));
	v.push_back(Cell(6, 7));
	v.push_back(Cell(7, 5));

	MiniMaxCell mmc = p.getEnemyCurrentMaxGrade(b, v, 'X');
	int t = mmc.getGradeForThisCell();
	std::cout << t;
}

