/*
 * Main.cpp
 *
 *  Created on: Dec 2, 2017
 *     Names : Yuval Weinstein & Ofir Ben Shoham.
 *     Id: 208580613 & 208642496.
 */
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include "Server.h"


using namespace std;

int main() {
    
    int portNumber;
    string line;
    ifstream myfile("exe/ConnectingDetails.txt");
    
    if (!myfile.is_open()) {
        cout << "Unable to open file";
        return -1;
    }
    // Otherwise, opened the file.
    
    getline(myfile, line); // no use of the ip adress in the server.
    
    getline(myfile, line);
    size_t posPort = line.find(":") + 1;
    if (posPort > 0) {
        string temp = line.substr(posPort);
        portNumber = atoi(temp.c_str());
    }
    myfile.close();
    
    Server server(portNumber); // the port we read from the file.
    server.socketInitialize();
    server.setAndStartServer();
    server.workWithClients();
    
    
    return 0;
}
