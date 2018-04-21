/*
 * TesterAIPlayer.cpp
 *
 *  Created on: Nov 24, 2017
 *      Author: ofir
 */
#include "Board.h"
#include "AIPlayer.h"

void testerGetEnemyCurrentGrade(AIPlayer AI) {
	Board b1, b2;
	vector<Cell> b1PossibleCells = b1.possibleCellsToAssign(
			AI.getAIplayerSign());
	vector<Cell> b2PossibleCells = b2.possibleCellsToAssign(
			AI.getAIplayerSign());

	if (AI.getEnemyCurrentGrade(b1, 'O') != AI.getEnemyCurrentGrade(b2, 'O')) {
		cout << "Test (1) failed\n";
	}

	if (AI.getEnemyCurrentGrade(b1, 'O') != AI.getEnemyCurrentGrade(b2, 'O')) {
		cout << "Test (2) failed\n";
	}

	b1.enterToBoard('X', 4, 4);

	if (AI.getEnemyCurrentGrade(b1, 'O') != -2) {
		cout << "Test (3) failed\n";
		b1.printBoard();
		cout << "But return: " << AI.getEnemyCurrentGrade(b1, 'O') << endl;
	}

	if (AI.getEnemyCurrentGrade(b1, 'O') != -AI.getEnemyCurrentGrade(b1, 'X')) {
		cout << "Test (4) failed\n";
	}

	b1.enterToBoard('X', 7, 4);
	b1.enterToBoard('X', 8, 4);

	if (AI.getEnemyCurrentGrade(b1, 'O') != -4) {
		cout << "Test (5) failed\n";
		b1.printBoard();
		cout << "But return: " << AI.getEnemyCurrentGrade(b1, 'O') << endl;
	}
	b1.enterToBoard('O', 7, 4);

	if (AI.getEnemyCurrentGrade(b1, 'O') != -2) {
		cout << "Test (6) failed\n";
		b1.printBoard();
		cout << "But return: " << AI.getEnemyCurrentGrade(b1, 'O') << endl;
	}

}

void TestAIChoose(AIPlayer AI) {
	Board b;
	b.enterToBoard('O', 4, 4);
	b.enterToBoard('X', 3, 4);
	b.enterToBoard('O', 3, 3);

	vector<Cell> possibleOfX;
	possibleOfX.push_back(Cell(3, 2));


	MiniMaxCell m = AI.getEnemyCurrentMaxGrade(b, possibleOfX, 'X'); // enemy is X
	int grade = m.getGradeForThisCell();
	if (grade != 3) {
		cout << "Test (7) failed\n need to be 3 but it's : " << grade << endl;
	}
	possibleOfX.clear();

	b.enterToBoard(' ', 3, 3);
	b.enterToBoard(' ', 6, 5);
	b.enterToBoard(' ', 3, 2);



	b.enterToBoard('X', 4, 4);
	b.enterToBoard('O', 5, 3);
	b.enterToBoard('O', 5, 4);
	b.enterToBoard('O', 5, 5);


	possibleOfX.push_back(Cell(6, 2));
	possibleOfX.push_back(Cell(6, 3));
	possibleOfX.push_back(Cell(6, 4));
	possibleOfX.push_back(Cell(6, 5));
	possibleOfX.push_back(Cell(6, 6));
	m = AI.getEnemyCurrentMaxGrade(b, possibleOfX, 'X');
	grade = m.getGradeForThisCell();


	if (grade != 3) {
		cout << "Test (8) failed\n need to be 3 but it's : " << grade << endl;
	}

	b.enterToBoard(' ', 5, 3);
	b.enterToBoard(' ', 6, 5);

	b.enterToBoard('O', 4, 5);
	b.enterToBoard('O', 5, 5);
	b.enterToBoard('X', 4, 4);






	possibleOfX.clear();
	b.enterToBoard('O', 3, 5);
	b.enterToBoard('X', 5, 4);
	//b.printBoard();

	possibleOfX.push_back(Cell(3, 6));
	possibleOfX.push_back(Cell(4, 6));
	possibleOfX.push_back(Cell(5, 6));
	m = AI.getEnemyCurrentMaxGrade(b, possibleOfX, 'X');
	grade = m.getGradeForThisCell();


	if (grade != 5) {
		cout << "Test (9) failed\n need to be 5 but it's : " << grade << endl;

	}


}

void startAITests(AIPlayer AI) {
	testerGetEnemyCurrentGrade(AI);
	TestAIChoose(AI);
}

/*int main() {
	AIPlayer AI('O');
	startAITests(AI);

}*/

