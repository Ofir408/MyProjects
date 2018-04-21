/*
 * RemotePlayer.cpp
 *
 *  Created on: Dec 8, 2017
 *      Author: Ofir Ben Shoham
 */

#include "RemotePlayer.h"
using namespace std;

RemotePlayer::RemotePlayer(Client myClient, GameLogic gl) {
	client = myClient;
	gameLogic = gl;

	if (myClient.getPlayerNum() == 1) {
		// x is the first player
		currentPlayer = 'X';
	} else {
		currentPlayer = 'O';
	}
}

void RemotePlayer::printPossibleMoves(Board& b, char Ps) {
	vector<Cell> possibleMovesVec = b.possibleCellsToAssign(Ps);
	cout << "Player: " << Ps << " ,Your Possible Moves are: ";
	for (unsigned i = 0; i < possibleMovesVec.size(); i++) {
		possibleMovesVec[i].printCell();
	}
	cout << " \nPlease choose a cell from your possible options,"
			" for example write 3 and press enter" << endl
			<< "                   4 and press enter:" << endl;
}

Cell RemotePlayer::chooseCell(Board* board, char playerSign) {

	// update his board before passing to the second player his turn.
	DoubleCell dc;
	Cell otherTurn, myTurn;
	if (board->possibleCellsToAssign(playerSign).empty()) {

		client.sendAndWriteToServer(board); // write here his turn and then read other player turn.
		return Cell(4,4);
	}

	if (playerSign == 'X') {
		// the first player first write his turn and then read
		printPossibleMoves(*board, playerSign);
		client.setPlayerNum(1);
		dc = client.sendAndWriteToServer(board); // write here his turn and then read other player turn.
	} else {
		client.setPlayerNum(2);
		// the second player first read other player sign and then write.
		dc = client.sendAndWriteToServer(board); // write here his turn and then read other player turn.
	}



	// I added it in order to check if we need to finish the socket.
	if (checkIfGetFinishCell(dc)) {
		close(client.getClientSocket()); // close the client socket.
		exit(0); // exit from the game.
	}

	myTurn = dc.getCurrentPlayerMove();
	otherTurn = dc.getOtherPlayerMove();

	// first make the other player turn.
	board->inputAssignManager(Cell::returnOtherSign(playerSign), otherTurn);

	//board->inputAssignManager(playerSign, dc.getCurrentPlayerMove());

	// return the current player turn - the cell that choosen (that he entered via input from user).
	return myTurn;
}

bool std::RemotePlayer::checkIfGetFinishCell(DoubleCell dc) {
	Cell firstCell(dc.getCurrentPlayerMove().getX() + 2, dc.getCurrentPlayerMove().getY() + 2);
	Cell secondCell(dc.getOtherPlayerMove().getX() + 2, dc.getOtherPlayerMove().getY() + 2);
	if (firstCell.getX() == 89 || firstCell.getY() == 89) {
		return true;
	}
	if (secondCell.getX() == 89 || secondCell.getY() == 89) {
		return true;
	}
	return false;
}
