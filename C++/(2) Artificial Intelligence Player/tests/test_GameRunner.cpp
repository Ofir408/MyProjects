#include "gtest/gtest.h"
#include "GameRunner.h"
#include "HumanPlayer.h"

Board createBoard1ForExample();
Board createBoard2ForExample();
Board createBoard3ForExample();

/*  Created on: Nov 27, 2017
 *  Author: Yuval Weinstein
 */

Board createBoard5ForExample() {
	Board b;
	b.setCell(3, 3, 'X');
	b.setCell(3, 4, 'X');
	b.setCell(4, 3, 'X');
	b.setCell(4, 4, 'X');
	return b;
}

TEST(GameRunnerTests, CanContinueTest) {
	Board b1 = createBoard1ForExample();
	Board b2 = createBoard2ForExample();
	Board b3 = createBoard3ForExample();
	Board b4 = createBoard5ForExample();
	HumanPlayer p1('X');
	HumanPlayer p2('O');
	GameRunner gr1(b1, p1, p2, 'X');
	GameRunner gr2(b2, p1, p2, 'X');
	GameRunner gr3(b3, p1, p2, 'X');
	GameRunner gr4(b4, p1, p2, 'X');

	EXPECT_TRUE(gr1.canToContinue());
	EXPECT_TRUE(gr2.canToContinue());
	EXPECT_FALSE(gr3.canToContinue());
	EXPECT_FALSE(gr4.canToContinue());
}

TEST(GameRunnerTests, TurnTest) {
	Board b1 = createBoard1ForExample();
	Board b2 = createBoard2ForExample();
	Board b3 = createBoard3ForExample();
	HumanPlayer p1('X');
	HumanPlayer p2('O');
	GameRunner gr1(b1, p1, p2, 'X');
	GameRunner gr2(b2, p1, p2, 'X');
	GameRunner gr3(b3, p1, p2, 'O');

	EXPECT_EQ(gr1.getCurrentPlayerTurn().getPlayerSign(), p1.getPlayerSign());
	EXPECT_EQ(gr2.getCurrentPlayerTurn().getPlayerSign(), p1.getPlayerSign());
	EXPECT_EQ(gr3.getCurrentPlayerTurn().getPlayerSign(), p2.getPlayerSign());
}




