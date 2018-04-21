/*
 * Client.h
 *
 *  Created on: Dec 6, 2017
 *      Author: ofir
 */

#ifndef CLIENT_H_
#define CLIENT_H_
#include "Cell.h"
#include "DoubleCell.h"
#include "Board.h"

namespace std {
    
    class Client {
    public:
        /**
         * constructor for the client obejct.
         * Gets:
         * const char *serverIP - ip adress.
         * int serverPort - number of the port of the server, which is common both for the client & the server.
         */
        Client(const char *serverIP, int serverPort);
        
        /**
         * default constructor.
         */
        Client() {
            DoubleCell d;
            clientSocket = 0;
            serverIP = 0;
            serverPort = 0;
            playerNum = 0;
            turnsToPlay = d;
        }
        
        /**
         * gets boolean to know if it's the first time of connection in order to wait to the second client.
         * Then, connect to the socket of the server in order to pass data between the client
         * and the server.
         */
        void connectToServer(bool firstClient = false);
        
        /**
         * send and write with the socket of the server, and returns DoubleCell
         * object which defines to hold two cells to play.
         * The first is other cell and the second it's the current cell to play.
         */
        DoubleCell sendAndWriteToServer(Board* board);
        
        /**
         * read the client num of this current client, which is 1 for player number 1
         * Or 2 for player number 2. Then, set it as a field of the client.
         */
        void readHisCilentNum();
        
        /**
         * read from the socket of the server the next move to play, and return it.
         */
        Cell getCurrentTurn();
        
        /**
         * gets row and col and then send them into the socket of our server.
         */
        void sendTurnToServer(int row, int col);
        
        /**
         * return the cell to play of this current client and not the other one.
         */
        Cell getMyCell() { return this->turnsToPlay.getCurrentPlayerMove(); }
        
        /**
         * return other cell to play.
         */
        Cell getOtherCell() { return this->turnsToPlay.getOtherPlayerMove(); }
        
        int getPlayerNum() const {
            return playerNum;
        }
        
        DoubleCell getTurnsToPlay() const {
            return turnsToPlay;
        }
        
        void setPlayerNum(int playerNum) {
            this->playerNum = playerNum;
        }
        
    private:
        const char *serverIP;
        int serverPort;
        int clientSocket;
        int playerNum;
        DoubleCell turnsToPlay;
        
    };
    /* namespace std */
}

#endif /* CLIENT_H_ */
