/*
 * RemotePlayer.h
 *
 *  Created on: Dec 8, 2017
 *      Author: ofir
 */

#ifndef REMOTEPLAYER_H_
#define REMOTEPLAYER_H_
#include "Client.h"
#include "GameRunner.h"

namespace std {
    
    class RemotePlayer : public Player {
    public:
        
        /**
         * constructor for remote player
         * Gets:
         * Client myClient - client of this remote player.
         * GameLogic gl - takes care for the logic in our game involved with remote player.
         */
        RemotePlayer(Client myClient, GameLogic gl);
        
        /**
         * implements the common function in abstract class "Player".
         * Gets pointer to board and player sign to choose the new empty cell for.
         * And return it.
         */
        Cell chooseCell(Board* board, char playerSign);
        
        /**
         * print possible cells for the player to play in his current turn.
         * The player sign is char playerSign that comes as an input.
         */
        void printPossibleMoves(Board &board, char playerSign);
        
        /**
         * returns the player sign of this remote player.
         */
        char getPlayerSign() { return this->currentPlayer; }
        
        //RemotePlayer(){}
    private:
        GameLogic gameLogic;
        Client client; // in order his socket will really change when return to RemotePlayer.
        char currentPlayer;
    };
    
} /* namespace std */

#endif /* REMOTEPLAYER_H_ */
