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
//#include <stdexcept>
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

	#ifdef _WIN32
#include <windows.h>
//#include <conio.h>
#endif
//Here is the #include for gyro*************************************************
#ifdef _WIN32
#include <GL/gl.h>
#include <GL/glu.h>
//#include <glut.h>
#elif __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <mach/clock.h>
#include <mach/mach.h>
#else
#include <GL/glut.h>
#endif
//#include <iostream>

#if __linux__|| __APPLE__
#include <unistd.h>
#endif
#include <cmath>

//#include "Iedk.h"
//#include "IedkErrorCode.h"
//#include "IEmoStateDLL.h"

//#include <iostream>
#include <fstream>
#include <memory>
#include <stdexcept>
//#include <thread>
#include <vector>
#include "bot_control.h"

using std::cout;
using std::cerr;
using std::unique_ptr;

#define PI 3.1418

bool  outOfBound = false;
float currX      = 0,
      currY      = 0;
float xmax       = 0,
      ymax       = 0,
      x          = 0;
float preX       = 0,
      preY       = 0;
int   incOrDec   = 10;
int   count      = 0;
float oldXVal    = 0,
      oldYVal    = 0;
double maxRadius = 10000;
unsigned long pass = 0,
              globalElapsed = 0;

const IEE_MotionDataChannel_t targetChannelList[] = {
    IMD_COUNTER,
    IMD_GYROX,
    IMD_GYROY,
    IMD_GYROZ,
    IMD_ACCX,
    IMD_ACCY,
    IMD_ACCZ,
    IMD_MAGX,
    IMD_MAGY,
    IMD_MAGZ,
    IMD_TIMESTAMP
};

const char header[] = "COUNTER, GYROX, GYROY, GYROZ, ACCX, ACCY, ACCZ, MAGX, "
"MAGY, MAGZ, TIMESTAMP";

char const * filename = "Gyro.csv";
std::ofstream ofs(filename, std::ios::trunc);

unique_ptr<void, void(*)(DataHandle)> hMotionData(IEE_MotionDataCreate(), &IEE_MotionDataFree);
//Gyro Functions***************************************************************************************
void showGyro(bool consoler)
{
    IEE_MotionDataUpdateHandle(0, hMotionData.get());
    unsigned int nSamplesTaken = 0;
    IEE_MotionDataGetNumberOfSample(hMotionData.get(), &nSamplesTaken);

    if (nSamplesTaken != 0) {
        std::vector<double> data(nSamplesTaken);
        for (int sampleIdx = 0; sampleIdx<(int)nSamplesTaken; ++sampleIdx) {
            for (int i = 0;
                i<sizeof(targetChannelList) / sizeof(IEE_MotionDataChannel_t);
                i++) {

                IEE_MotionDataGet(hMotionData.get(), targetChannelList[i],
                    data.data(), data.size());
                if (i == 1 && consoler)
                    std::cout << "GyroX =" << data[sampleIdx];
                if (i == 2 && consoler)
                {
                    std::cout << "GyroY =" << data[sampleIdx];
                    std::cout << std::endl;
                }

                ofs << data[sampleIdx] << ",";
            }

            ofs.flush();
            ofs << std::endl;
        }       
    }
}
/*
void init(void) 
{
   glClearColor (0.0, 0.0, 0.0, 0.0);
   glShadeModel (GL_FLAT);
}

void drawCircle( float Radius, int numPoints )
{
  glBegin( GL_LINE_STRIP );
    for( int i=0; i<numPoints; i++ )
    {
		float Angle = i * (2.0*PI/numPoints); // use 360 instead of 2.0*PI if
		float X = cos( Angle )*Radius;  // you use d_cos and d_sin
		float Y = sin( Angle )*Radius;
		glVertex2f( X, Y );		
	}
  glEnd();
}

void drawFilledCircle(float radius)
{
	glEnable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glPointSize(radius);
	//glPoint(0, 0, 0); 
}

void display(void)
{
   glClear(GL_COLOR_BUFFER_BIT);
   glPushMatrix();
  
   glColor3f(1.0,1.0,1.0);
   drawCircle(800,100);
   glColor3f(0.0,0.0,1.0);
   drawCircle(maxRadius-4000,800);
   glColor3f(0.0,1.0,1.0);
   drawCircle(maxRadius,1000);
	
  
   glColor3f(1.0, 0.0, 0.0);   
   glRectf(currX-400.0, currY-400.0, currX+400.0, currY+400.0);
   
   glPopMatrix();
   glutSwapBuffers();
}
*/
void changeXY(int x) // x = 0 : idle
{
    if( currX >0 )
    {
        float temp = currY/currX;
        currX -= incOrDec;
        currY = temp*currX;
    }
    else if( currX < 0)
    {
        float temp = currY/currX;
        currX += incOrDec;
        currY = temp*currX;
    }
    else
    {
        if( currY > 0 ) currY -= incOrDec;
        else if( currY <0 ) currY += incOrDec;
    }
    if( x == 0)
    {
        if( (std::abs(currX) <= incOrDec) && (std::abs(currY) <= incOrDec))
        {
            xmax = 0;
            ymax = 0;
        }
        else
        {
            xmax = currX;
            ymax = currY;
        }
    }
    else
    {
        if( (std::abs(currX) <= incOrDec) && (std::abs(currY) <= incOrDec))
        {
            xmax = 0;
            ymax = 0;
        }
    }
}


void updateDisplay(void)
{	  
    showGyro(false);

   int gyroX = 0, gyroY = 0;
   IEE_HeadsetGetGyroDelta(0, &gyroX, &gyroY);
   xmax += gyroX;
   ymax += gyroY;

   if( outOfBound )
   {
	   if( preX != gyroX && preY != gyroY )
	   {
		   xmax = currX;
		   ymax = currY;
	   }
   }

   double val = sqrt((float)(xmax*xmax + ymax*ymax));
  
    std::cout <<"xmax : " << xmax <<" ; ymax : " << ymax << std::endl;

   
   if( val >= maxRadius )
   {
	   changeXY(1);	
	   outOfBound = true;
	   preX = gyroX;
	   preY = gyroY;
   }
   else
   {		
	   outOfBound = false;
		if(oldXVal == gyroX && oldYVal == gyroY)
		{
			++count;
			if( count > 10 )
			{									
				changeXY(0);
			}
		}
		else
		{
			count = 0;
			currX = xmax;
			currY = ymax;
			oldXVal = gyroX;
			oldYVal = gyroY;			
		}
		
   }
}
/*
#ifdef _WIN32
      Sleep(15);
#endif
#if __linux__ || __APPLE__
      usleep(10000);
#endif
   glutPostRedisplay(); 
}*/

/*
void reshape(int w, int h)
{
   glViewport (0, 0, (GLsizei) w, (GLsizei) h);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glOrtho(-50000.0, 50000.0, -50000.0, 50000.0, -1.0, 1.0);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}
void mouse(int button, int state, int x, int y) 
{
   switch (button) {
      case GLUT_LEFT_BUTTON:
         if (state == GLUT_DOWN)
            glutIdleFunc(updateDisplay);
         break;
      case GLUT_MIDDLE_BUTTON:
         if (state == GLUT_DOWN)
            glutIdleFunc(NULL);
         break;
      default:
         break;
   } 	
}
*/
#if __linux__
double GetTickCount()
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now))
      return 0;
    return now.tv_sec * 1000.0 + now.tv_nsec / 1000000.0;
}
#endif
#ifdef __APPLE__
double GetTickCount()
{
    struct timespec ts;
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    ts.tv_sec = mts.tv_sec;
    ts.tv_nsec = mts.tv_nsec;
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1000000.0;
}
#endif
#define PORT "COM3"
#define BAUDRATE 9600
/* 
 *  Request double buffer display mode.
 *  Register mouse input callback functions
 */
#include "tserial.h"
//************************************************************************************************

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

		//gyro
		IEE_MotionDataSetBufferSizeInSec(1);   


		i=0;
		bool changed = false;
		int ready = false;
		int update = false;
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
					//updateDisplay();

					//exit(0);
					break;
				}
				changed = false;
			}
			if(i==10){
				updateDisplay();
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
					ready = true;
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
					if(update && i==10){
						//updateDisplay();
					}
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

			// gyro code
			if(!ready) continue;

		int gyroX = 0, gyroY = 0;
		int err = IEE_HeadsetGetGyroDelta(userID, &gyroX, &gyroY);        
		
		if(!update){
			if (err == EDK_OK){
				std::cout << std::endl;
				std::cout << "You can move your head now." << std::endl;

	#ifdef _WIN32
				Sleep(1000);
	#endif
	#if __linux__ || __APPLE__
				usleep(10000);
	#endif
				update = true;
				//break;
			}else if (err == EDK_GYRO_NOT_CALIBRATED){
				std::cout << "Gyro is not calibrated. Please stay still." << std::endl;
				showGyro(true);
			}else if (err == EDK_CANNOT_ACQUIRE_DATA){
				std::cout << "Cannot acquire data" << std::endl;
				showGyro(true);
			}else{
				std::cout << "No headset is connected" << std::endl;
			}

	#ifdef _WIN32
		Sleep(15);
	#endif
	#if __linux__ || __APPLE__
			usleep(15000);
	#endif
			}
		}
	}
    catch (const std::runtime_error& e) {
		std::cerr << e.what() << std::endl;
		std::cout << "Press any keys to exit..." << std::endl;
		getchar();
	}

	
#ifdef _WIN32
   globalElapsed = GetTickCount();
#endif
#if __linux__|| __APPLE__
   globalElapsed = ( unsigned long ) GetTickCount();
#endif
	/*
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
   glutInitWindowSize (650, 650); 
   glutInitWindowPosition (100, 100);
   glutCreateWindow (argv[0]);
   init ();
   glutDisplayFunc(display); 
   glutReshapeFunc(reshape); 
   glutIdleFunc(updateDisplay);
   glutMainLoop();  
   */  
  /* while(1){
	   //Sleep(15);
	   updateDisplay();
   }*/
   ofs.close();

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
