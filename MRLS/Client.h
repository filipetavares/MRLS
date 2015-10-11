#ifndef CLIENT_H  
#define CLIENT_H 

#pragma once

#include <winsock2.h>
#include <iostream>
#include <thread>
#include <json.h>
#include "Tracker.h"

#define MAX_BUFF_SIZE 65500 //65535

class Tracker;

class Client
{

private:
	int ID = -1; //By default clients start without ID, the ID is atributed by the server according to the existent clients
	//int Position[3]; // (x, y , theta)
	float markerSize = -1;

	std::thread ReadImageThread;
	std::thread ReadSensorThread; //threads that are constantly reading the information sent by the device

	SOCKET DataSocket = INVALID_SOCKET;
	SOCKET ImageSocket = INVALID_SOCKET;

public:
	float Position[3]; // (x, y , theta)
	Tracker* tracker;

#pragma region TESTING VARIABLES TO ERASE
	long timestamp = 0;
	float gyro[3];
	double gyroTS = -1;
	float accel[3];
	double accelTS = -1;
	float orientation[3]; //(z,x,y) (azimth,pitch,roll);
	float kalmanPos[3];
	int wifiRssi = 0;
	int wifiFreq = 0;
	char * image = NULL;
	int ImageSize = -1;
	string intrisincsFile;
	float distanceFromMarker = 0;
	int markerID = -1;
	float markerPosition[3];
	double samplingAverage = 0;
	double samplingMAX = 0;
	double samplingMIN = 9999999999;
	
	double input[10][3][4];

#pragma endregion

	Client();
	Client(SOCKET socketTCP, SOCKET socketUDP, int ID, float x, float y, float theta, string IntrinsicsFile, float markerSize);
	~Client();

	void start();

	int getID();

	float* getPosition();

	//void setSocket(SOCKET socket); ??
	SOCKET getTCPSocket();
	SOCKET getUDPSocket();

	void closeSocket(SOCKET* socket);

	int Client::TCPReceive(SOCKET s, char ** ptr, int packetSize = -1);
	char* Client::UDPReceive(SOCKET socket);
	//char* UDPReceive
	//void TCPSend
	//void UDPSend

	void ReadImageFromNetwork(); //this will be a threadble function that keeps trying to read the image from the network;
	void ReadSensorDataFromNetwork();

	void DisplayImage();

	void startTracking();

	void initInput();
	void DebugInput(int i);
};

#endif