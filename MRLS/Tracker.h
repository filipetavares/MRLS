#ifndef TRACKER_H 
#define TRACKER_H

#pragma once
#include "aruco.h"
#include "cvdrawingutils.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <map>
#include <fstream>


#include "Client.h"
#include "eMarker.h"
#include "ConsoleLogger.h"

class Client;
void write_text_to_log_file(const std::string &text, const std::string &filename, bool &fileStarted);

class Tracker
{
public:
	int KalmanPS = 0; //Kalman per second
	int MTPS = 0; //Marker tracking per second
	int MTIPS = 40; //how many markerTracking iteration/s we want;
	int KFIPS = 100; // how many kalman filter iterations/s we want;

	Tracker();
	Tracker(Client * client, string intrinsicsFile = "");
	~Tracker();

	void start();

	void markerTracking();

	void displayImage(Client * client);
	void Tracker::decodeImage(char* imgSrc, int imgSrcSz);

	void getIntrinsicsFromFile(string FilePath);
	void initiateMarkersDB();
	void setMarkerSize(float size);

private:
	Client* client;
	CConsoleLoggerEx LoggingConsole;
	std::thread trackerThread;
	std::thread getImageFromClientThread;
	std::thread KalmanThread;
	std::thread TESTINGKALMAN;
	aruco::Marker DrawingMarker;


	std::map<int, void*> markerDB;
	bool EXIT_FLAG = false; //set to true to stop the tracking threads
	cv::Mat rawImage;
	bool ORIENTATION_TO_Y = true; //we only track in a 2D world, so this defines if we are rotating on Y or Z.
	bool STARTED = false; // FLAG to know if the tracker has already started;
	std::map<std::string, bool> logFiles;

	vector<float> MarkerPosition; //
	bool MARKERTRACKING_FLAG = false; // so that we know when we have an available position from the markertracking.

	cv::Mat trackingImage;


	//OpenCV/Aruco Variables
	aruco::MarkerDetector MarkerDetector;
	aruco::CameraParameters TheCameraParameters;


	float MarkerSize = -1;

	//Kalman Variables
	cv::KalmanFilter KF;
	cv::Mat_<float> state;
	cv::Mat processNoise;
	vector<cv::Point> positionVector, KalmanVector;


	void calcCameraPosition(vector<aruco::Marker> markers, vector<float> &data);

	void getImageFromClient();
	void copyRawImage(cv::Mat* dst);
	void KalmanCorrection();
	void ExtendedKalmanFilter1();
	void ExtendedKalmanFilter2();
	void ExtendedKalmanFilter3();

};

#endif