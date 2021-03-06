/****************************************************************************
**
** Copyright 2015 by Emotiv. All rights reserved
** Example - Mental Command Demo
** This example demonstrates how the user’s conscious mental intention can be
** recognized by the Mental Command TM detection and used to control the
** movement of a 3D virtual object.
** It also shows the steps required to train the Mental Command
** suite to recognize distinct mental actions for an individual user.
**
****************************************************************************/

#include <iostream>
#include <map>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <cstdlib>
#include <stdio.h>

#include "IEmoStateDLL.h"
#include "Iedk.h"
#include "IedkErrorCode.h"
#include "Socket.h"
#include "MentalCommandControl.h"

#ifdef _WIN32
#include <conio.h>
#pragma comment(lib, "Ws2_32.lib")
#endif
#if __linux__ || __APPLE__
    #include <unistd.h>
    #include <termios.h>
    int _kbhit(void);
    int _getch( void );
#endif

void sendMentalCommandAnimation(SocketClient& sock, EmoStateHandle eState);
void handleMentalCommandEvent(std::ostream& os,
                              EmoEngineEventHandle cognitivEvent);
bool handleUserInput();
void promptUser();

int i;

int main(int argc, char** argv) {

	// location of the machine running the 3D motion cube
	std::string receiverHost = "localhost";
	
	if (argc > 2) {
		std::cout << "Usage: " << argv[0] << " <hostname>" << std::endl;
        std::cout << "The arguments specify the host of the motion cube"
                     " (Default: localhost)" << std::endl;
		return 1;
	}

	if (argc > 1) {
		receiverHost = std::string(argv[1]);
	}

	EmoEngineEventHandle eEvent	= IEE_EmoEngineEventCreate();
	EmoStateHandle eState		= IEE_EmoStateCreate();
	unsigned int userID			= 0;
	
	try {
        /*Connect with EmoEngine*/
        if (IEE_EngineConnect() != EDK_OK) {
            throw std::runtime_error("Emotiv Driver start up failed.");
        }
        /*************************************************************/
        /*Connect with Emocomposer app on port 1726*/
        /*Connect with Control Panel app on port 3008*/
//        if (IEE_EngineRemoteConnect("127.0.0.1",1726) != EDK_OK) {
//            throw std::runtime_error("Emotiv Driver start up failed.");
//        }
        /*************************************************************/
		else {
			std::cout << "Emotiv Driver started!" << std::endl;
		}

		int startSendPort = 20000;
		std::map<unsigned int, SocketClient> socketMap;

        std::cout << "Type \"exit\" to quit, \"help\" to list available commands..."
                  << std::endl;
		promptUser();

		i=0;
		bool changed = false;
		while (true) {
			
			// Handle the user input
			/*if (_kbhit()) {
				if (!handleUserInput()) {
					break;
				}
			}*/
			if(changed){
				if(i!=10)i++;
				switch(i){
				case 1:
					parseCommand("set_actions 0 push", std::cout);					
					break;
				case 2:
					parseCommand("training_action 0 neutral", std::cout);
					break;
				case 3:
					parseCommand("training_start 0", std::cout);
					break;
				case 4:
					break;
				case 5:
					parseCommand("training_accept 0", std::cout);
					break;
				case 6:
					parseCommand("training_action 0 push", std::cout);
					break;
				case 7:
					parseCommand("training_start 0", std::cout);
					break;
				case 8:
					break;
				case 9:
					parseCommand("training_accept 0", std::cout);
					break;
				case 10:
					//exit(0);
					break;
				}
				changed = false;
			}

			int state = IEE_EngineGetNextEvent(eEvent);

			// New event needs to be handled
			if(i==2){
				//cout << "bahar "  << endl;
			}
			if (state == EDK_OK) {
				//cout << "\t\t\tandar "<< endl;

				IEE_Event_t eventType = IEE_EmoEngineEventGetType(eEvent);
				IEE_EmoEngineEventGetUserId(eEvent, &userID);
				//cout << " 00000000 *** \t" <<  eventType << endl;
				switch (eventType) {

                // New headset connected
                // create a new socket to send the animation
                case IEE_UserAdded:
                {

					changed = true;
                    std::cout << std::endl << "New user " << userID
                              << " added, sending MentalCommand animation to ";
                    std::cout << receiverHost << ":" << startSendPort << "..."
                              << std::endl;
                    promptUser();

                    socketMap.insert(std::pair<unsigned int, SocketClient>(
                        userID, SocketClient(receiverHost, startSendPort, UDP)));

                    startSendPort++;
                    break;
                }

                // Headset disconnected, remove the existing socket
                case IEE_UserRemoved:
                {
                    std::cout << std::endl << "User " << userID
                              << " has been removed." << std::endl;
                    promptUser();

                    std::map<unsigned int, SocketClient>::iterator iter;
                    iter = socketMap.find(userID);
                    if (iter != socketMap.end()) {
                        socketMap.erase(iter);
                    }
                    break;
                }

                // Send the MentalCommand animation if EmoState has been updated
                case IEE_EmoStateUpdated:
                {
					//cout << "state updated " << endl;
					if(i==2 || i==6)
						changed = true;
                    IEE_EmoEngineEventGetEmoState(eEvent, eState);

                    std::map<unsigned int, SocketClient>::iterator iter;
                    iter = socketMap.find(userID);
                    if (iter != socketMap.end()) {
                        sendMentalCommandAnimation(iter->second, eState);
                    }
                    break;
                }

                // Handle MentalCommand training related event
                case IEE_MentalCommandEvent:
                {
					changed = true;
					printf("handle mental command\n");
                    handleMentalCommandEvent(std::cout, eEvent);
                    break;
                }

                default:
                    break;
				}
			}
			else if (state != EDK_NO_EVENT) {
				std::cout << "Internal error in Emotiv Engine!" << std::endl;
				break;
			}

#ifdef _WIN32
    Sleep(15);
#endif
#if __linux__ || __APPLE__
        usleep(15000);
#endif
		}
	}
    catch (const std::runtime_error& e) {
		std::cerr << e.what() << std::endl;
		std::cout << "Press any keys to exit..." << std::endl;
		getchar();
	}

	IEE_EngineDisconnect();
	IEE_EmoStateFree(eState);
	IEE_EmoEngineEventFree(eEvent);

	return 0;
}


void sendMentalCommandAnimation(SocketClient& sock, EmoStateHandle eState) {

	std::ostringstream os;

    IEE_MentalCommandAction_t actionType	=
            IS_MentalCommandGetCurrentAction(eState);
    float	actionPower = IS_MentalCommandGetCurrentActionPower(eState);

    os << static_cast<int>(actionType) << ","
       << static_cast<int>(actionPower*100.0f);

	sock.SendBytes(os.str());
	if(i==10)cout << os.str() << endl;
}


void handleMentalCommandEvent(std::ostream& os,
                              EmoEngineEventHandle cognitivEvent) {

	unsigned int userID = 0;
	IEE_EmoEngineEventGetUserId(cognitivEvent, &userID);
    IEE_MentalCommandEvent_t eventType =
            IEE_MentalCommandEventGetType(cognitivEvent);

	switch (eventType) {

    case IEE_MentalCommandTrainingStarted:
    {
        os << std::endl << "MentalCommand training for user " << userID
           << " STARTED!" << std::endl;
        break;
    }

    case IEE_MentalCommandTrainingSucceeded:
    {
        os << std::endl << "MentalCommand training for user " << userID
           << " SUCCIEEDED!" << std::endl;
        break;
    }

    case IEE_MentalCommandTrainingFailed:
    {
        os << std::endl << "MentalCommand training for user " << userID
           << " FAILED!" << std::endl;
        break;
    }

    case IEE_MentalCommandTrainingCompleted:
    {
        os << std::endl << "MentalCommand training for user " << userID
           << " COMPLETED!" << std::endl;
        break;
    }

    case IEE_MentalCommandTrainingDataErased:
    {
        os << std::endl << "MentalCommand training data for user " << userID
           << " ERASED!" << std::endl;
        break;
    }

    case IEE_MentalCommandTrainingRejected:
    {
        os << std::endl << "MentalCommand training for user " << userID
           << " REJECTED!" << std::endl;
        break;
    }

    case IEE_MentalCommandTrainingReset:
    {
        os << std::endl << "MentalCommand training for user " << userID
           << " RESET!" << std::endl;
        break;
    }

    case IEE_MentalCommandAutoSamplingNeutralCompleted:
    {
        os << std::endl << "MentalCommand auto sampling neutral for user "
           << userID << " COMPLETED!" << std::endl;
        break;
    }

    case IEE_MentalCommandSignatureUpdated:
    {
        os << std::endl << "MentalCommand signature for user " << userID
           << " UPDATED!" << std::endl;
        break;
    }

    case IEE_MentalCommandNoEvent:
        break;

    default:
        //@@ unhandled case
        assert(0);
        break;
	}
    promptUser();
}


bool handleUserInput() {

	static std::string inputBuffer;

	char c = _getch();


#if __linux__ || __APPLE__
    if ((c == '\n'))
#else
    if (c == '\r')
#endif
    {
		std::cout << std::endl;
		std::string command;

		const size_t len = inputBuffer.length();
		command.reserve(len);

		// Convert the input to lower case first
		for (size_t i=0; i < len; i++) {
			command.append(1, tolower(inputBuffer.at(i)));
		}

		inputBuffer.clear();

		bool success = parseCommand(command, std::cout);
        promptUser();
		return success;
	}
	else {
		if (c == '\b') { // Backspace key
			if (inputBuffer.length()) {
				putchar(c);
				putchar(' ');
				putchar(c);
				inputBuffer.erase(inputBuffer.end()-1);
			}
		}
		else {
			inputBuffer.append(1,c);
			std::cout << c;
		}
	}	

	return true;
}

void promptUser()
{
	std::cout << "MentalCommandDemo> ";
}

#ifdef __linux__
int _kbhit(void)
{
    struct timeval tv;
    fd_set read_fd;

    tv.tv_sec=0;
    tv.tv_usec=0;

    FD_ZERO(&read_fd);
    FD_SET(0,&read_fd);

    if(select(1, &read_fd,NULL, NULL, &tv) == -1)
    return 0;

    if(FD_ISSET(0,&read_fd))
        return 1;

    return 0;
}

int _getch( void )
{
   struct termios oldattr, newattr;
   int ch;

   tcgetattr( STDIN_FILENO, &oldattr );
   newattr = oldattr;
   newattr.c_lflag &= ~( ICANON | ECHO );
   tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
   ch = getchar();
   tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );

   return ch;
}
#endif
#ifdef __APPLE__
int _kbhit (void)
{
    struct timeval tv;
    fd_set rdfs;

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    FD_ZERO(&rdfs);
    FD_SET (STDIN_FILENO, &rdfs);

    select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);
}

int _getch( void )
{
    int r;
    unsigned char c;
    if((r = read(0, &c, sizeof(c))) < 0 )
    {
        return r;
    }
    else
    {
        return c;
    }
}
#endif
