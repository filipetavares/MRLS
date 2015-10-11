#ifndef TESTINGCLIENT_H  
#define TESTINGCLIENT_H 

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <map>

using namespace std;

//void write_text_to_log_file(const std::string &text, const std::string &filename, bool &fileStarted);

class TestingClient
{
public:
	TestingClient();
	~TestingClient();

	void reRun(std::string logFileName);
	void readFile(std::string path);
	void ExtendedKalmanFilter(string logFileName = "testLogFile.txt");
	void write_text_to_log_file(const std::string &text, const std::string &filename, bool &fileStarted);

	bool EXIT_FLAG = false;


	string currentFile = "";
	float IPS = 1000;						//Iterations per Second (default = 100) (nolim = 1000)
	float lowPassFilterVar = 0.7;// 0.3;		// Low Pass filter in case it is used (set 1 to turn off).
	float accelErrorWindow = 0.15;// 0.15;
	float gyroErrorWindow = 0.1; 
	float accelError = 0.00022;//0.00022;				//Accelerator sensor Measurment Error
	float gyroError = 0.03;					//Gyro sensor measurment error

	float Position[3]; // (x, y , theta)
	long timestamp = 0;
	float gyro[3];
	double gyroTS = -1;
	float accel[3];
	double accelTS = -1;
	float orientation[3]; //(z,x,y) (azimth,pitch,roll);
	float kalmanPos[3];
	float markerPosition[3];

	std::map<std::string, bool> logFiles;

	std::vector<double> accelDeltaTime;
	std::vector<std::array<float, 3>> acceleration;
	std::vector<std::array<float, 2>> velocity;
	std::vector<std::array<float, 2>> position;
	std::vector<std::array<float, 2>> predictedPosition;
	std::vector<std::array<float, 3>> realAcceleration;
	std::vector<double> gyroDeltaTime;
	std::vector<float> angularVelocity;
	std::vector<float> angle;
	std::vector<std::array<float, 3>> realAngularVelocity;
	std::vector<std::array<float, 3>> inclination;
	std::vector<int> markerTracked;
	std::vector<std::array<float, 3>> Marker;
	std::vector<double> accelTimestamp;
	std::vector<double> gyroTimestamp;
	std::vector<std::array<float, 3>> rawAcceleration;
	std::vector<std::array<float, 3>> rawGyro;




};


#endif