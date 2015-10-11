#include "stdafx.h"
#include "Client.h"
#include <chrono>


Client::Client()
{
}


Client::Client(SOCKET socketData, SOCKET socketImage, int ID, float x, float y, float theta, string IntrinsicsFile, float MarkerSize)
{
	DataSocket = socketData;
	ImageSocket = socketImage;
	Client::ID = ID;
	Position[0] = x;
	Position[1] = y;
	Position[2] = theta;
	kalmanPos[0] = x;
	kalmanPos[1] = y;
	kalmanPos[2] = theta;
	accel[0] = 0;
	accel[1] = 0;
	accel[2] = 0;
	gyro[0] = 0;
	gyro[1] = 0;
	gyro[2] = 0;
	timestamp = 0;
	intrisincsFile = IntrinsicsFile;
	markerSize = MarkerSize;
	
	initInput();

	tracker = new Tracker(this, IntrinsicsFile);
	tracker->setMarkerSize(markerSize);
}

Client::~Client()
{
	closeSocket(&DataSocket);
	closeSocket(&ImageSocket);
	delete tracker;
	if (image != NULL)
	delete image;
	ReadSensorThread.join();
	ReadImageThread.join();
}

void Client::start()
{
	ReadSensorThread = std::thread(&Client::ReadSensorDataFromNetwork, this);
	ReadImageThread = std::thread(&Client::ReadImageFromNetwork, this);
	tracker->start();
}

void Client::startTracking()
{
	tracker->start();
}


int Client::getID()
{
	return ID;
}

float* Client::getPosition()
{
	return Position;
}

SOCKET Client::getTCPSocket()
{
	return DataSocket;
}

SOCKET Client::getUDPSocket()
{
	return ImageSocket;
}

void Client::closeSocket(SOCKET* socket)
{
	if (*socket != INVALID_SOCKET)
	{
		closesocket(*socket);
		*socket = INVALID_SOCKET;
	}
}

int Client::TCPReceive(SOCKET s, char ** ptr, int packetSize)
{
	char buff[MAX_BUFF_SIZE]; //creates a buffer to receive the data
	int iResult = recv(s, buff, MAX_BUFF_SIZE, 0); //receives the 1st packet

	if (iResult > 0 && packetSize < 0 || iResult > 0 && iResult == packetSize) //if the packet is recevied and it is complete
	{
		*ptr = new char[iResult]; //creates a container for the received data
		std::copy(buff, buff + iResult, *ptr); // copies the valid recived data
		return iResult;
	}
	else if (iResult > 0 && packetSize > iResult) //if the packet is received and it is not complete
	{
		int packetCounter = 0;
		*ptr = new char[packetSize]; //alloc enough memory for the whole data
		std::copy(buff, buff + iResult, *ptr); //copy the 1st part of the data
		packetCounter += iResult; // keep track of how much data has been received
		while (packetCounter < packetSize) //while the data received is less then the expected data keep doing this
		{
			iResult = recv(s, buff, MAX_BUFF_SIZE, 0); //receive the next packet
			if (iResult + packetCounter > packetSize)
				break;
			else if (iResult > 0) //if a packet is received
			{	
				std::copy(buff, buff+iResult, *ptr + packetCounter); //keep filling the container
				packetCounter += iResult; // update the amount of data received
			}
			else //if a packet is not received or is not valid
			{
				delete *ptr;
				*ptr = NULL;
				return -1;
			}

		}
		if (packetCounter == packetSize) // if the data has the expected size
			return packetSize;
		else
		{
			delete *ptr;
			*ptr = NULL;
			return -1;
		}
	}
	else //if we dont receive a packet or if the packet is not valid
	{
		delete *ptr;
		*ptr = NULL;
		return -1;
	}

	
}

//char* Client::UDPReceive(SOCKET s)
//{
//	char buff[MAX_BUFF_SIZE];
//	sockaddr_in service;
//	int serviceLen = sizeof(service);
//
//	int iResult = recvfrom(s, buff, MAX_BUFF_SIZE, 0, (struct sockaddr *)&service, &serviceLen);
//	if (iResult > 0)
//	{
//		char* temp = new char[iResult];
//		std::copy(buff, buff + iResult, temp);
//		return temp;
//	}
//	return NULL;
//}

void Client::ReadSensorDataFromNetwork()
{
	while (DataSocket != INVALID_SOCKET)
	{
		char* msg = NULL;
		int iResult = TCPReceive(DataSocket, &msg);
		if (msg == NULL)
			continue;

		vector<string> JsonMsgs;
		char buffer[1000];
		int buffsize = 0;
		int parentesisCounter = 0;
		for (int i = 0; i < iResult; i++)
		{
			if (msg[i] == '{')
			{
				parentesisCounter++;
				buffer[buffsize] = msg[i];
				buffsize++;
			}
			else if (msg[i] == '}' && parentesisCounter > 0)
			{
				parentesisCounter--;
				buffer[buffsize] = msg[i];
				buffsize++;

				if (parentesisCounter == 0)
				{
					char* temp = new char[buffsize];
					memcpy(temp, buffer, buffsize);
					JsonMsgs.push_back(string(temp));
					delete[] temp;
					buffsize = 0;
				}

			}
			else if (parentesisCounter >= 1)
			{
				buffer[buffsize] = msg[i];
				buffsize++;
			}
		}
		delete[] msg;

		for (int i = 0; i < JsonMsgs.size(); i++)
		{	
			Json::Value parsedMsg;
			Json::Reader jsonReader;
			bool parsed = jsonReader.parse(JsonMsgs[i], parsedMsg);
			if (parsed)
			{
				gyro[0] = parsedMsg["SensorList"]["Gyro"][0].asFloat();
				gyro[1] = parsedMsg["SensorList"]["Gyro"][1].asFloat();
				gyro[2] = parsedMsg["SensorList"]["Gyro"][2].asFloat();
				gyroTS = parsedMsg["SensorList"]["GyroTS"].asLargestInt();
				accel[0] = parsedMsg["SensorList"]["Accel"][0].asFloat();
				accel[1] = parsedMsg["SensorList"]["Accel"][1].asFloat();
				accel[2] = parsedMsg["SensorList"]["Accel"][2].asFloat();
				accelTS = parsedMsg["SensorList"]["AccelTS"].asLargestInt();
				orientation[0] = parsedMsg["SensorList"]["Orientation"][0].asFloat();
				orientation[1] = parsedMsg["SensorList"]["Orientation"][1].asFloat();
				orientation[2] = parsedMsg["SensorList"]["Orientation"][2].asFloat();
				wifiRssi = parsedMsg["SensorList"]["Wifi"].asInt();
				wifiFreq = parsedMsg["SensorList"]["WifiFrequency"].asInt();
				timestamp = parsedMsg["Timestamp"].asLargestInt();
			}
		}
	}
}

void Client::ReadImageFromNetwork()
{
	
	while (ImageSocket != INVALID_SOCKET)
	{
		int len;
		recv(ImageSocket, (char*)&len, sizeof(int), NULL);
		int leng = ntohl(len);
		std::string stringlen = std::to_string(leng) + "\n";

		if (leng > 200000)
			leng = -1;
		//send(ImageSocket, stringlen.c_str(), stringlen.size(), 0);

		char* img = NULL;
		ImageSize = TCPReceive(ImageSocket, &img, leng);
		
		//send(ImageSocket, "\n", 2, 0);

		if (img == NULL)
			continue;

		char *temp = image;
		image = img;

		if (temp != NULL)
		delete[] temp;
	}
}

void Client::DisplayImage()
{
	tracker->displayImage(this);
}

void Client::DebugInput(int i)
{
	gyro[0] = input[i][0][0];
	gyro[1] = input[i][0][1];
	gyro[2] = input[i][0][2];
	gyroTS = input[i][0][3];
	accel[0] = input[i][1][0];
	accel[1] = input[i][1][1];
	accel[2] = input[i][1][2];
	accelTS = input[i][1][3];
	orientation[0] = input[i][2][0];
	orientation[1] = input[i][2][1];
	orientation[2] = input[i][2][2];
}

void Client::initInput()
{
	double array[10][3][4] = {
		{
			{ 0, 0, 0, 0 },
			{ 0, 0, 0, 1000000000 },
			{ 0, 0, 90 * CV_PI / 180, 0 }
		},
		{
			{ 0, 0, 0, 0 },
			{ 2, 0, 0, 2000000000 },
			{ 0, 0, 90 * CV_PI / 180, 0 }
		},
		{
			{ 0, 0, 0, 0 },
			{ -2, 0, 0, 3000000000 },
			{ 0, 0, 90 * CV_PI / 180, 0 }
		},
		{
			{ 0, 0, 0, 0 },
			{ 0, 0, 0, 4000000000 },
			{ 0, 0, 90 * CV_PI / 180, 0 }
		},
		{
			{ 0, 0, 0, 0 },
			{ 0, 0, 0, 5000000000 },
			{ 0, 0, 90 * CV_PI / 180, 0 }
		},
		{
			{ 0, 0, 0, 0 },
			{ -2, 0, 0, 6000000000 },
			{ 0, 0, 90 * CV_PI / 180, 0 }
		},
		{
			{ 0, 0, 0, 0 },
			{ 2, 0, 0, 7000000000 },
			{ 0, 0, 90 * CV_PI / 180, 0 }
		},
		{
			{ 0, 0, 0, 0 },
			{ 0, 0, 0, 8000000000 },
			{ 0, 0, 90 * CV_PI / 180, 0 }
		},
		{
			{ 0, 0, 0, 0 },
			{ 0, 0, 0, 9000000000 },
			{ 0, 0, 90 * CV_PI / 180, 0 }
		},
		{
			{ 0, 0, 0, 0 },
			{ 0, 0, 0, 10000000000 },
			{ 0, 0, 90 * CV_PI / 180, 0 }
		}
	};

	for (int i = 0; i < 10 ; i++)
	for (int j = 0; j < 3; j++)
	for (int k = 0; k < 4; k++)
		input[i][j][k] = array[i][j][k];
}

