#include "stdafx.h"
#include "Tracker.h"
#include <mutex>

std::mutex mtx;


int CV_ErrorHandler(int status, const char* func_name,
	const char* err_msg, const char* file_name,
	int line, void* userdata); //handler created to supress opencv error messages

Tracker::Tracker()
{
	
}

Tracker::Tracker(Client* c, string intrinsicsFile)
{
	client = c;
	getIntrinsicsFromFile(intrinsicsFile);
}


Tracker::~Tracker()
{
	EXIT_FLAG = true;
	
	for (auto iter = markerDB.begin(); iter != markerDB.end(); iter++) //terminates all the clients
	{
		eMarker* m = (eMarker*)(iter->second);
		delete m;
	}

	if (getImageFromClientThread.joinable())
		getImageFromClientThread.join();
	if (trackerThread.joinable())
		trackerThread.join();
	if (KalmanThread.joinable())
		KalmanThread.join();

	if (TESTINGKALMAN.joinable())
		TESTINGKALMAN.join();

	LoggingConsole.Close();
}

#pragma region CLASS_FUNCTIONS
void Tracker::start()
{
	if (!STARTED)
	{
		initiateMarkersDB();

		getImageFromClientThread = std::thread(&Tracker::getImageFromClient, this); // copies the images from the client
		trackerThread = std::thread(&Tracker::markerTracking, this); //markerDetection
		KalmanThread = std::thread(&Tracker::ExtendedKalmanFilter3, this); // Kalman filter//std::thread(&Tracker::KalmanCorrection, this); // Kalman filter

		//TESTINGKALMAN = std::thread(&Tracker::KalmanCorrection, this);

		STARTED = true;
	}
	else
		cout << "Tracker is already running!" << endl;
}



void Tracker::setMarkerSize(float ms)
{
	MarkerSize = ms;
}

void Tracker::getImageFromClient()
{
	const int FRAMES_PER_SECOND = 30;
	const int SKIP_TICKS = 1000 / FRAMES_PER_SECOND;

	DWORD next_game_tick = GetTickCount();
	int sleep_time = 0;

	while (!EXIT_FLAG)
	{
		decodeImage(client->image, client->ImageSize);

		next_game_tick += SKIP_TICKS;
		sleep_time = next_game_tick - GetTickCount();
		if (sleep_time >= 0)
		{
			//std::cout << "Sleep Time " << sleep_time  << std::endl;
			Sleep(sleep_time);
		}
	}
}

void Tracker::copyRawImage(cv::Mat* dst)
{
	std::lock_guard<std::mutex> guard(mtx);
	rawImage.copyTo(*dst);

}


void Tracker::displayImage(Client * client)
{
	cvNamedWindow("Image Display", CV_WINDOW_AUTOSIZE);
	//vector<uchar> ImageBuffer;
	IplImage img;


	while (cv::waitKey(5) != 'q')
	{
		try
		{
			if (rawImage.size > 0)//(client->ImageSize > 0)
			{
				//ImageBuffer.resize(client->ImageSize);
				//memcpy((char*)(&ImageBuffer[0]), client->image, client->ImageSize);
				cv::Mat jpeg;
				//jpeg = cv::imdecode(cv::Mat(ImageBuffer), CV_LOAD_IMAGE_COLOR);
				//cvtColor(jpeg, jpeg, CV_BGR2RGB);
				//decodeImage(client->image, client->ImageSize);
				copyRawImage(&jpeg);
				
				DrawingMarker.draw(jpeg, cv::Scalar(0, 0, 255), 1);
				aruco::CvDrawingUtils::draw3dAxis(jpeg, DrawingMarker, TheCameraParameters);
				img = jpeg;
				cvRedirectError(CV_ErrorHandler);
				cvShowImage("Image Display", &img);
			}
		}
		catch (std::exception &e)
		{
			cout << e.what() << endl;
			continue;
		}

	}

	cvDestroyWindow("Image Display");
}

int CV_ErrorHandler(int status, const char* func_name,
	const char* err_msg, const char* file_name,
	int line, void* userdata)
{
	//supress the error messages;
	return 0;
}

void Tracker::getIntrinsicsFromFile(string filePath)
{
	if (filePath != "")
	{
		try
		{
			TheCameraParameters.readFromXMLFile(filePath);
		}
		catch (std::exception &e)
		{
			cout << "File: " << filePath << " not found!" << endl;
		}
	}
}

void Tracker::initiateMarkersDB()
{
	markerDB.insert(std::pair<int, eMarker*>(114, new eMarker(114, 64.5, 146, 0)));
	markerDB.insert(std::pair<int, eMarker*>(115, new eMarker(115, 69, 349, 0)));
	markerDB.insert(std::pair<int, eMarker*>(231, new eMarker(231, 311, 390, 0)));
	markerDB.insert(std::pair<int, eMarker*>(71, new eMarker(71, 309, 226, 0)));
	markerDB.insert(std::pair<int, eMarker*>(108, new eMarker(227, 309, 102, 0)));
}

void Tracker::decodeImage(char* imgSrc, int imgSrcSz)
{
	std::lock_guard<std::mutex> guard(mtx);
	if (imgSrcSz > 0)
	{
		try
		{
			vector<uchar> ImageBuffer;
			ImageBuffer.resize(imgSrcSz);
			memcpy((char*)(&ImageBuffer[0]), imgSrc, imgSrcSz);
			rawImage = cv::imdecode(cv::Mat(ImageBuffer), CV_LOAD_IMAGE_GRAYSCALE);
			//rawImage = rawImage.t();
			//cv::flip(rawImage, rawImage, 1);
			//cvtColor(rawImage, rawImage, CV_BGR2RGB);
		}
		catch (std::exception &e)
		{
			cout << "DECODE IMAGE -> " << e.what() << endl;
		}
	}
}
#pragma endregion

#pragma region MARKERTRACKING
void Tracker::markerTracking()
{
	MarkerPosition = { 0, 0, 0 };
	while (rawImage.dims == 0 && !EXIT_FLAG) //waits for a image to start the tracking;
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(200)); //checks 5 times per sec if there is a image or if it is time to quit
	}
	if (EXIT_FLAG)
		return;

	cv::Mat imageCopy;
	copyRawImage(&imageCopy);

	TheCameraParameters.resize(imageCopy.size());
	MarkerDetector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);
	MarkerDetector.setThresholdParams(13, 6);
	
	vector<aruco::Marker> markerVector;
	vector<float> positionData = { 0, 0, 0 };

	const int FRAMES_PER_SECOND = MTIPS;
	const int SKIP_TICKS = 1000 / FRAMES_PER_SECOND;

	DWORD next_game_tick = GetTickCount();
	int sleep_time = 0;
	DWORD startCount = GetTickCount();
	int fps = 0;

	while (!EXIT_FLAG)
	{
		fps++;
		next_game_tick += SKIP_TICKS;
		sleep_time = next_game_tick - GetTickCount();
		if (sleep_time >= 0)
		{
			//std::cout << "Sleep Time " << sleep_time  << std::endl;
			Sleep(sleep_time);
		}

		if (GetTickCount() - startCount > 1000)
		{
			this->MTPS = fps;
			startCount = GetTickCount();
			fps = 0;
		}


		positionData = { 0, 0, 0 };
		copyRawImage(&imageCopy);
		try
		{
			MarkerDetector.detect(imageCopy, markerVector, TheCameraParameters, MarkerSize);
		}
		catch (std::exception &e)
		{
			cout << "MARKER TRAKING -> " << e.what() << endl;
			continue;
		}
		

		if (markerVector.size() <= 0)
			continue;

		calcCameraPosition(markerVector, positionData);

		if (markerVector.size() > 0)
			DrawingMarker = markerVector[0];

		client->Position[0] = positionData[0];
		client->Position[1] = positionData[1];
		client->Position[2] = positionData[2];

		client->distanceFromMarker = std::sqrt((markerVector[0].Tvec.at<float>(0, 0) * markerVector[0].Tvec.at<float>(0, 0))
												+ (markerVector[0].Tvec.at<float>(1, 0) * markerVector[0].Tvec.at<float>(1, 0))
												+ (markerVector[0].Tvec.at<float>(2, 0) * markerVector[0].Tvec.at<float>(2, 0)));

		client->markerID = markerVector[0].id;

		MarkerPosition = positionData;

		MARKERTRACKING_FLAG = true;
	}

	return;

}

#pragma region Helping_Funcs
void Tracker::calcCameraPosition(vector<aruco::Marker> markers, vector<float> &data)
{
	cv::Mat R33(3, 3, CV_32FC1); // Rodrigues Rotation matrix
	cv::Mat R44(4, 4, CV_32FC1); // Rotation matrix + Translation Vector (see description below) #1

	float mcos, msin;
	eMarker* foundMarker;

	vector<float> temp = { 0, 0 };
	int validMarkers = 0;


	for (int i = 0; i < markers.size(); i++)
	{

		auto iter = markerDB.find(markers[i].id);
		if (iter == markerDB.end())
		{
			continue;
		}

		foundMarker = (eMarker*)iter->second;

		cv::Rodrigues(markers[i].Rvec, R33);

		//#1
		//|R11	R12 R13 T11|
		//|R21	R22 R23 T12|
		//|R31	R32	R33	T13|
		//|0	0	0	1  |

		R44.at<float>(0, 0) = R33.at<float>(0, 0);
		R44.at<float>(0, 1) = R33.at<float>(0, 1);
		R44.at<float>(0, 2) = R33.at<float>(0, 2);
		R44.at<float>(1, 0) = R33.at<float>(1, 0);
		R44.at<float>(1, 1) = R33.at<float>(1, 1);
		R44.at<float>(1, 2) = R33.at<float>(1, 2);
		R44.at<float>(2, 0) = R33.at<float>(2, 0);
		R44.at<float>(2, 1) = R33.at<float>(2, 1);
		R44.at<float>(2, 2) = R33.at<float>(2, 2);

		R44.at<float>(0, 3) = markers[i].Tvec.at<float>(0, 0);
		R44.at<float>(1, 3) = markers[i].Tvec.at<float>(1, 0);
		R44.at<float>(2, 3) = markers[i].Tvec.at<float>(2, 0);

		R44.at<float>(3, 0) = 0;
		R44.at<float>(3, 1) = 0;
		R44.at<float>(3, 2) = 0;
		R44.at<float>(3, 3) = 1;

		cv::Mat R44I = R44.inv();

		// | x | - | x*cos(angle) - y*sin(angle) |
		// | y | - | x*sin(angle) + y*cos(angle) |
		mcos = std::cos(-foundMarker->angle * CV_PI / 180);
		msin = std::sin(-foundMarker->angle * CV_PI / 180);

		float RX, RY;

		if (ORIENTATION_TO_Y)
		{
			RX = R44I.at<float>(0, 3) * mcos - R44I.at<float>(1, 3) * msin;
			RY = R44I.at<float>(0, 3) * msin + R44I.at<float>(1, 3) * mcos;
		}
		else
		{
			RX = R44I.at<float>(0, 3) * mcos - R44I.at<float>(2, 3) * msin;
			RY = R44I.at<float>(0, 3) * msin + R44I.at<float>(2, 3) * mcos;
		}

		//TODO: MUDAR ISTO PORQUE ESTAMOS A FAZER UMA MÈDIA DA MEDIDA DOS MARKERS...o KALMAN FAZ ISTO AUTOMATICAMENTE...
		data[0] += foundMarker->x + RX;
		data[1] += foundMarker->y + RY;


		//eulerRotVect[0] = cv::fastAtan2(R44I.at<float>(2, 1), R44I.at<float>(2, 2)); //-> theta x = atan2(R32, R33)
		//eulerRotVect[1] = cv::fastAtan2(-R44I.at<float>(2, 0),
		//	sqrt((R44I.at<float>(2, 1) * R44I.at<float>(2, 1)) +
		//	(R44I.at<float>(2, 2) * R44I.at<float>(2, 2)))); //-> theta y = atan2(-R31, sqrt((R32)^2 + (R33)^2))
		//eulerRotVect[2] = cv::fastAtan2(R44I.at<float>(1, 0), R44I.at<float>(0, 0));

		if (ORIENTATION_TO_Y)
		{
			data[2] = cv::fastAtan2(R44I.at<float>(1, 0), R44I.at<float>(0, 0));
			data[2] = fmod((data[2] + foundMarker->angle), 360);
		}
		else
		{
			data[2] = cv::fastAtan2(-R44I.at<float>(2, 0),
				sqrt((R44I.at<float>(2, 1) * R44I.at<float>(2, 1)) +
				(R44I.at<float>(2, 2) * R44I.at<float>(2, 2)))); //-> theta y = atan2(-R31, sqrt((R32)^2 + (R33)^2))

			data[2] = fmod((data[2] + foundMarker->angle), 360);

		}

		temp[0] += std::cos(data[2] * CV_PI / 180);
		temp[1] += std::sin(data[2] * CV_PI / 180);
		validMarkers++;
	}

	if (validMarkers > 0)
	{
		data[0] = data[0] / validMarkers;
		data[1] = data[1] / validMarkers;
		data[2] = cv::fastAtan2(temp[1] / validMarkers, temp[0] / validMarkers); // angle average avg = atan2(SUM(sin(a))/n, SUM(cos(a))/n)
	}

}
#pragma endregion
#pragma endregion

#pragma region KALMAN FILTER
void Tracker::KalmanCorrection()
{
	logFiles.insert(std::pair<std::string, boolean>("marker_log_file.txt", false));

	KF = cv::KalmanFilter(2, 2, 0);
	state = cv::Mat_<float>(2, 1);
	processNoise = cv::Mat(2, 1, CV_32F);

	cv::setIdentity(KF.measurementMatrix);
	cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-3));
	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(0.1));
	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(0.1));

	cv::Mat_<float> measurments(2, 1);
	vector <cv::Point> posv, kalmanv;
	posv.clear();
	kalmanv.clear();

	const int FRAMES_PER_SECOND = 60;
	const int SKIP_TICKS = 1000 / FRAMES_PER_SECOND;
	DWORD next_game_tick = GetTickCount();
	int sleep_time = 0;
	bool game_is_running = true;
	DWORD startCount = GetTickCount();

	long lastaccelts = client->accelTS - 1;
	


	while (!EXIT_FLAG)
	{
		long accelTS = client->accelTS;
		float acceldeltaTime = ((accelTS - lastaccelts) / 1000000000.0f);

		//if (MARKERTRACKING_FLAG)
		//{
			cv::Mat prediction = KF.predict();
			cv::Point predictPt(prediction.at<float>(0), prediction.at<float>(1));

			measurments(0) = MarkerPosition[0];
			measurments(1) = MarkerPosition[1];

			cv::Point measPt(measurments(0), measurments(1));
			posv.push_back(measPt);

			cv::Mat estimated = KF.correct(measurments);
			cv::Point statePt(estimated.at<float>(0), estimated.at<float>(1));

			kalmanv.push_back(statePt);

			client->kalmanPos[0] = estimated.at<float>(0);
			client->kalmanPos[1] = estimated.at<float>(1);
			client->kalmanPos[2] = MarkerPosition[2];

			//MARKERTRACKING_FLAG = false;

			string logText("" +to_string(acceldeltaTime) + " " + to_string(KF.statePre.at<float>(0, 0)) + " " + to_string(KF.statePre.at<float>(1, 0))); //ts x' y'
			auto iter = logFiles.find("marker_log_file.txt");
			write_text_to_log_file(logText, iter->first, (iter->second));

			lastaccelts = accelTS;
		//}

		next_game_tick += SKIP_TICKS;
		sleep_time = next_game_tick - GetTickCount();
		if (sleep_time >= 0)
		{
			//std::cout << "Sleep Time " << sleep_time  << std::endl;
			Sleep(sleep_time);
		}
	}
}

//void Tracker::ExtendedKalmanFilter1()
//{
//
//	//TESTING BLOCK----------
//
//	//bool debugInput = false;
//	//int debugInputVar = 0;
//	//client->DebugInput(0);
//
//	logFiles.insert(std::pair<std::string, boolean>("marker_log_file.txt", false));
//
//	KF = cv::KalmanFilter(2, 2, 0);
//	state = cv::Mat_<float>(2, 1);
//	processNoise = cv::Mat(2, 1, CV_32F);
//
//	cv::setIdentity(KF.measurementMatrix);
//	cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-3));
//	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(0.1));
//	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(0.1));
//
//	cv::Mat_<float> measurments(2, 1);
//	vector <cv::Point> posv, kalmanv;
//	posv.clear();
//	kalmanv.clear();
//
//	//-----------------------
//
//
//	bool LOG_FILE_DEBUG = true;
//	
//	if (LOG_FILE_DEBUG)
//		logFiles.insert(std::pair<std::string, boolean>("log_file.txt", false));
//
//	float lowPassFilterVar = 0.2;
//
//	cv::KalmanFilter KFx = cv::KalmanFilter(2, 2, 1); // [x, vx] [1marker, wifi, (sensors?)] [ax(accelerometer)]
//	cv::Mat Ax = cv::Mat_<float>(2, 2);
//	cv::Mat Bx = cv::Mat_<float>(2, 1);
//	cv::Mat Ux = cv::Mat_<float>(1, 1);
//	cv::Mat Zx = cv::Mat_<float>(2, 1);
//	cv::Mat Hx = cv::Mat_<float>(2, 2);
//	float prevXAccel = 0;
//
//
//	cv::KalmanFilter KFy = cv::KalmanFilter(2, 2, 1);
//	cv::Mat Ay = cv::Mat_<float>(2, 2);
//	cv::Mat By = cv::Mat_<float>(2, 1);
//	cv::Mat Uy = cv::Mat_<float>(1, 1);
//	cv::Mat Zy = cv::Mat_<float>(2, 1);
//	cv::Mat Hy = cv::Mat_<float>(2, 2);
//	float prevYAccel = 0;
//
//	cv::KalmanFilter KFo = cv::KalmanFilter(1, 1, 1);
//	cv::Mat Ao = cv::Mat_<float>(1, 1);
//	cv::Mat Bo = cv::Mat_<float>(1, 1);
//	cv::Mat Uo = cv::Mat_<float>(1, 1);
//	cv::Mat Zo = cv::Mat_<float>(1, 1);
//	cv::Mat Ho = cv::Mat_<float>(1, 1);
//
//	cv::Mat estimatedo = cv::Mat_<float>(1, 1);
//	estimatedo.at<float>(0, 0) = 0;
//
//	bool USE_IMAGE = false;
//	double lastimestamp = client->timestamp - 1;
//	double lastgyrots = client->gyroTS - 1;
//	double lastaccelts = client->accelTS - 1;
//	float accelError = 0.00022;
//	float gyroError = 0.03;
//	float LastAngleInRads = 0;
//
//	/*int counter = 0;
//	clock_t startTime = clock();
//	clock_t endTime = clock();
//	clock_t beginTime;*/
//
//	const int FRAMES_PER_SECOND = KFIPS;
//	const int SKIP_TICKS = 1000 / FRAMES_PER_SECOND;
//
//	DWORD next_game_tick = GetTickCount();
//	int sleep_time = 0;
//	bool game_is_running = true;
//	DWORD startCount = GetTickCount();
//	int fps = 0;
//	KFo.statePost.at<float>(0, 0) = 0;
//
//	float accel[3] = { client->accel[0], client->accel[1], client->accel[2] };
//	float lastValidAccel[3] = { 0, 0, 0 };
//	float accelErrorWindow = 0.5;
//	float gyro[3] = { client->gyro[0], client->gyro[1], client->gyro[2] };
//	float lastValidGyro[3] = { 0, 0, 0 };
//	float orientation[3] = { 0, 0, 0 };
//	float gyroErrorWindow = 0.05;
//
//	int recallCounterx = 0;
//	int xTries = 5;
//	int recallCountery = 0;
//	int yTries = 5;
//
//	while (!EXIT_FLAG)
//	{
//
//		//if (debugInput && debugInput < 10)
//		//{
//		//	client->DebugInput(debugInputVar);
//		//	debugInputVar++;
//		//}
//			
//
//		if (GetTickCount() - startCount > 1000)
//		{
//			this->KalmanPS = fps;
//			startCount = GetTickCount();
//			fps = 0;
//		}
//
//		double gyroTS = client->gyroTS;
//		double accelTS = client->accelTS;
//
//
//		double gyrodeltaTime = ((gyroTS - lastgyrots) / 1000000000.0f);
//		double acceldeltaTime = ((accelTS - lastaccelts) / 1000000000.0f);
//
//
//		if (gyrodeltaTime > 0)
//		{
//			if (abs(lastValidGyro[0] - client->gyro[0]) <= gyroErrorWindow)
//			{
//				gyro[0] = 0;
//
//			}
//			else
//			{
//				gyro[0] = client->gyro[0];
//				lastValidGyro[0] = gyro[0];
//			}
//			if (abs(lastValidGyro[1] - client->gyro[1]) <= gyroErrorWindow)
//			{
//				gyro[1] = 0;
//			}
//			else
//			{
//				gyro[1] = client->gyro[1];
//				lastValidGyro[1] = gyro[2];
//			}
//			if (abs(lastValidGyro[2] - client->gyro[2]) <= gyroErrorWindow)
//			{
//				gyro[2] = 0;
//			}
//			else
//			{
//				gyro[2] = client->gyro[2];
//				lastValidGyro[2] = gyro[2];
//			}
//		}
//
//
//		orientation[0] = client->orientation[0]; //x
//		orientation[1] = client->orientation[2]; //y 
//		orientation[2] = client->orientation[1]; //z 
//
//		float temp[3] = { 0, 0, 0 };
//		float temp2[3] = { 0, 0, 0 };
//		temp2[0] = client->accel[0];
//		temp2[1] = client->accel[1];
//		temp2[2] = client->accel[2];
//
//		temp[0] = temp2[0] * cos(orientation[1])* cos(orientation[2]) + temp2[1] * cos(orientation[1])*sin(orientation[2]) + temp2[2] * sin(orientation[1]);
//		temp[1] = temp2[0] * -sin(orientation[2]) + temp2[1] * cos(orientation[2]);
//		temp[2] = temp2[0] * sin(orientation[1])* cos(orientation[2]) + temp2[1] * sin(orientation[1])*sin(orientation[2]) + temp2[2] * cos(orientation[1]);
//
//		if (acceldeltaTime > 0)
//		{
//			if (abs(lastValidAccel[0] - temp[0]) <= accelErrorWindow)
//			{
//				accel[0] = 0;
//			}
//			else
//			{
//				accel[0] = temp[0];
//				lastValidAccel[0] = accel[0];
//			}
//			if (abs(lastValidAccel[1] - temp[1]) <= accelErrorWindow)
//			{
//				accel[1] = 0;
//				recallCounterx++;
//			}
//			else
//			{
//				accel[1] = temp[1];
//				lastValidAccel[1] = accel[2];
//				recallCounterx = 0;
//			}
//			if (abs(lastValidAccel[2] - temp[2]) <= accelErrorWindow)
//			{
//				accel[2] = 0;
//				recallCountery++;
//
//			}
//			else
//			{
//				accel[2] = temp[2];
//				lastValidAccel[2] = accel[2];
//				recallCountery = 0;
//			}
//		}
//
//
//		//if (recallCounterx > 20)
//		//{
//		//	KFx.statePre.at<float>(1, 0) = 0;
//		//	recallCounterx = 0;
//		//}
//		//if (recallCountery > 20)
//		//{
//		//	KFy.statePre.at<float>(1, 0) = 0;
//		//	recallCountery = 0;
//		//}
//
//		//orientation[0] = client->orientation[0]; //x
//		//orientation[1] = client->orientation[2]; //y 
//		//orientation[2] = client->orientation[1]; //z 
//
//		//float temp[3] = { 0, 0, 0 };
//		//temp[0] = accel[0] * cos(orientation[1])* cos(orientation[2]) + accel[1] * cos(orientation[1])*sin(orientation[2]) + accel[2] * sin(orientation[1]);
//		//temp[1] = accel[0] * -sin(orientation[2]) + accel[1] * cos(orientation[2]);
//		//temp[2] = accel[0] * sin(orientation[1])* cos(orientation[2]) + accel[1] * sin(orientation[1])*sin(orientation[2]) + accel[2] * cos(orientation[1]);
//
//		//accel[0] = temp[0];
//		//accel[1] = temp[1];
//		//accel[2] = temp[2];
//
//		if (MARKERTRACKING_FLAG)
//			USE_IMAGE = true;
//
//
//
//		// TESTING BLOCK -----------------------------
//		if (USE_IMAGE)
//		{
//
//			cv::Mat prediction = KF.predict();
//			cv::Point predictPt(prediction.at<float>(0), prediction.at<float>(1));
//
//			measurments(0) = MarkerPosition[0];
//			measurments(1) = MarkerPosition[1];
//
//			cv::Point measPt(measurments(0), measurments(1));
//			posv.push_back(measPt);
//
//			cv::Mat estimated = KF.correct(measurments);
//			cv::Point statePt(estimated.at<float>(0), estimated.at<float>(1));
//
//			kalmanv.push_back(statePt);
//
//			/*client->kalmanPos[0] = estimated.at<float>(0);
//			client->kalmanPos[1] = estimated.at<float>(1);
//			client->kalmanPos[2] = MarkerPosition[2];*/
//		}
//		//--------------------------------------------
//
//
//
//		//KFo kalman -------
//
//		if (gyrodeltaTime > 0 || MARKERTRACKING_FLAG)
//		{
//			float PreviousAngle = KFo.statePost.at<float>(0, 0) * 180 / CV_PI;
//			int TimesAround = PreviousAngle / 360;
//			float MarkerAngle = MarkerPosition[2];
//			float nearAngle = MarkerAngle + TimesAround * 360;
//			float correctedMarkerAngle = 0;
//			if (abs(PreviousAngle - (nearAngle + 360)) <= abs(PreviousAngle - nearAngle))
//			{
//				correctedMarkerAngle = nearAngle + 360;
//			}
//			else
//			{
//				if (abs(PreviousAngle - (nearAngle - 360)) <= abs(PreviousAngle - nearAngle))
//					correctedMarkerAngle = nearAngle - 360;
//				else
//					correctedMarkerAngle = nearAngle;
//			}
//
//
//			Ao.at<float>(0, 0) = 1;
//			Bo.at<float>(0, 0) = gyrodeltaTime;
//			Zo.at<float>(0, 0) = correctedMarkerAngle * CV_PI / 180;
//			Uo.at<float>(0, 0) = gyro[0]; // rads/s
//
//			if (USE_IMAGE)
//				Ho.at<float>(0, 0) = 1;
//			else
//				Ho.at<float>(0, 0) = 0;
//
//			KFo.transitionMatrix = Ao;
//			KFo.controlMatrix = Bo;
//			KFo.measurementMatrix = Ho;
//
//			cv::setIdentity(KFo.processNoiseCov, cv::Scalar(gyroError*CV_PI / 180)); //Q
//			cv::setIdentity(KFo.measurementNoiseCov, cv::Scalar(1)); //R
//
//			cv::Mat predictiono = KFo.predict(Uo);
//			cv::Mat estimatedo = KFo.correct(Zo);
//
//			//cout << "Xo^ =" << KFo.statePre << endl;
//			//cout << "Xo^+1 =" << KFo.statePost << endl;
//			//cout << "K = " << KFo.gain << endl << endl;;
//
//			float radToDegrees = estimatedo.at<float>(0, 0) * 180 / CV_PI;
//			client->kalmanPos[2] = fmod(radToDegrees, 360);
//			lastgyrots = gyroTS;
//		}
//
//
//		
//		if (acceldeltaTime > 0 || MARKERTRACKING_FLAG)
//		{
//			//KFx Kalman ------
//			Ax.at<float>(0, 0) = 1;
//			Ax.at<float>(0, 1) = /*cos(client->kalmanPos[2] * CV_PI / 180)**//*cos(estimatedo.at<float>(0, 0))**/acceldeltaTime;
//			Ax.at<float>(1, 0) = 0;
//			if (recallCounterx > xTries)
//			{
//				Ax.at<float>(1, 1) = 0;
//				//recallCounterx = 0;
//			}
//			else
//				Ax.at<float>(1, 1) = 1;
//
//			Bx.at<float>(0, 0) = 0.5*/*cos(client->kalmanPos[2] * CV_PI / 180)**//*cos(estimatedo.at<float>(0, 0))**/pow(acceldeltaTime, 2);
//			Bx.at<float>(1, 0) = /*cos(client->kalmanPos[2] * CV_PI / 180)**//*cos(estimatedo.at<float>(0, 0))**/acceldeltaTime;
//
//			Zx.at<float>(0, 0) = MarkerPosition[0];
//			Zx.at<float>(1, 0) = 0;// ver como é com o beacon
//
//			//Ux.at<float>(0, 0) = -accel[1] * 100;
//
//			//Ux.at<float>(0, 0) = (-accel[1] * cos(estimatedo.at<float>(0, 0)) + accel[2] * sin(estimatedo.at<float>(0, 0))) * 100;
//
//			prevXAccel = prevXAccel*(1 - lowPassFilterVar) + (-accel[1])*lowPassFilterVar;
//			Ux.at<float>(0, 0) = prevXAccel * 100;
//
//			if (USE_IMAGE)
//				Hx.at<float>(0, 0) = 1;
//			else
//				Hx.at<float>(0, 0) = 0;
//			Hx.at<float>(0, 1) = 0;
//			Hx.at<float>(1, 0) = 0; // Resolve this to the beacon
//			Hx.at<float>(1, 1) = 0;
//
//			KFx.transitionMatrix = Ax;
//			KFx.controlMatrix = Bx;
//			KFx.measurementMatrix = Hx;
//
//			cv::setIdentity(KFx.processNoiseCov, cv::Scalar(accelError)); //Q
//			cv::setIdentity(KFx.measurementNoiseCov, cv::Scalar(1));
//
//			cv::Mat predictionx = KFx.predict(Ux);
//			cv::Mat estimatedx = KFx.correct(Zx);
//
//			//cout << "-----------VARIABLES-----------" << endl;
//			//cout << "Ax =\n" << KFx.transitionMatrix << endl;
//			//cout << "Bx =\n" << KFx.controlMatrix << endl;
//			//cout << "Ux =\n" << Ux << endl;
//			//cout << "Zx =\n" << Zx << endl;
//			//cout << "Qx =\n" << KFx.processNoiseCov << endl;
//			//cout << "Rx =\n" << KFx.measurementNoiseCov << endl;
//
//			//cout << "-----------PREDICTION-----------" << endl;
//			//cout << "Xx^ =\n" << KFx.statePre << endl;
//			//cout << "Px^ =\n" << KFx.errorCovPre << endl;
//
//			//cout << "-----------UPDATE-----------" << endl;
//			//cout << "Xx^+1 =\n" << KFx.statePost << endl;
//			//cout << "K =\n " << KFx.gain << endl << endl;
//			//cout << "Px^+1 =\n" << KFx.errorCovPost << endl;
//
//			//KFy kalman --------
//			Ay.at<float>(0, 0) = 1;
//			Ay.at<float>(0, 1) = /*sin(client->kalmanPos[2] * CV_PI / 180)*//*-sin(estimatedo.at<float>(0, 0))**/acceldeltaTime;
//			Ay.at<float>(1, 0) = 0;
//
//			if (recallCountery > yTries)
//			{
//				Ay.at<float>(1, 1) = 0;
//				//recallCountery = 0;
//			}
//			else
//				Ay.at<float>(1, 1) = 1;
//
//			By.at<float>(0, 0) = 0.5*/*sin(client->kalmanPos[2] * CV_PI / 180)*//*-sin(estimatedo.at<float>(0, 0))**/pow(acceldeltaTime, 2);
//			By.at<float>(1, 0) = /*sin(client->kalmanPos[2] * CV_PI / 180)*//*-sin(estimatedo.at<float>(0, 0))**/acceldeltaTime;
//
//			Zy.at<float>(0, 0) = MarkerPosition[1];
//			Zy.at<float>(1, 0) = 0;// ver como é com o beacon
//
//			//Uy.at<float>(0, 0) = accel[2] * 100;
//
//			//Uy.at<float>(0, 0) = (accel[2] * cos(estimatedo.at<float>(0, 0)) - (-accel[1])*sin(estimatedo.at<float>(0, 0))) * 100;
//
//			prevYAccel = prevYAccel*(1 - lowPassFilterVar) + (accel[2])*lowPassFilterVar;
//			Uy.at<float>(0, 0) = prevYAccel * 100;
//
//			if (USE_IMAGE)
//				Hy.at<float>(0, 0) = 1;
//			else
//				Hy.at<float>(0, 0) = 0;
//			Hy.at<float>(0, 1) = 0;
//			Hy.at<float>(1, 0) = 0; // Resolve this to the beacon
//			Hy.at<float>(1, 1) = 0;
//
//			KFy.transitionMatrix = Ay;
//			KFy.controlMatrix = By;
//			KFy.measurementMatrix = Hy;
//
//			cv::setIdentity(KFy.processNoiseCov, cv::Scalar(accelError)); //Q
//			cv::setIdentity(KFy.measurementNoiseCov, cv::Scalar(1));
//
//			cv::Mat predictiony = KFy.predict(Uy);
//			cv::Mat estimatedy = KFy.correct(Zy);
//
//
//			//cout << "-----------VARIABLES-----------" << endl;
//			//cout << "Ay =\n" << KFy.transitionMatrix << endl;
//			//cout << "By =\n" << KFy.controlMatrix << endl;
//			//cout << "Uy =\n" << Uy << endl;
//			//cout << "Zy =\n" << Zy << endl;
//			//cout << "Qy =\n" << KFy.processNoiseCov << endl;
//			//cout << "Ry =\n" << KFy.measurementNoiseCov << endl;
//
//			//cout << "-----------PREDICTION-----------" << endl;
//			//cout << "Xy^ =\n" << KFy.statePre << endl;
//			//cout << "Py^ =\n" << KFy.errorCovPre << endl;
//
//			//cout << "-----------UPDATE-----------" << endl;
//			//cout << "Xy^+1 =\n" << KFy.statePost << endl;
//			//cout << "K =\n " << KFy.gain << endl << endl;
//			//cout << "Py^+1 =\n" << KFy.errorCovPost << endl << endl;
//
//
//			//RETREIVE RESULTS--------------
//			client->kalmanPos[0] = estimatedx.at<float>(0, 0);
//			client->kalmanPos[1] = estimatedy.at<float>(0, 0);
//			lastaccelts = accelTS;
//
//			if (acceldeltaTime > 0)
//			{
//				if (LOG_FILE_DEBUG)
//				{
//					string logText("" + to_string(acceldeltaTime) + " "
//						+ to_string(accel[0]) + " " + to_string(Ux.at<float>(0, 0)) + " " + to_string(Uy.at<float>(0, 0)) + " "
//						+ to_string(estimatedx.at<float>(1, 0)) + " " + to_string(estimatedy.at<float>(1, 0)) + " "
//						+ to_string(estimatedx.at<float>(0, 0)) + " " + to_string(estimatedy.at<float>(0, 0))  + " " + to_string(KFx.statePre.at<float>(0, 0)) + " "
//						+ to_string(KFy.statePre.at<float>(0, 0)) + " " + to_string(MARKERTRACKING_FLAG ? 1 : 0)); //ts ax ay az vx vy x y x' y' marker
//
//					//string logText("" + to_string(acceldeltaTime) + " "
//					//	+ to_string(KFx.statePre.at<float>(0, 0)) + " " + to_string(KFy.statePre.at<float>(0, 0)) + " " + to_string(USE_IMAGE ? 1 : 0)); //ts x' y'
//					auto iter = logFiles.find("log_file.txt");
//					write_text_to_log_file(logText, iter->first, (iter->second));
//
//					//TESTING BLOCK --------------------------------
//					string logText2("" + to_string(acceldeltaTime) + " " + to_string(KF.statePre.at<float>(0, 0)) + " " + to_string(KF.statePre.at<float>(1, 0))); //ts x' y'
//					auto iter2 = logFiles.find("marker_log_file.txt");
//					write_text_to_log_file(logText2, iter2->first, (iter2->second));
//					//----------------------------------------------
//				}
//					
//			}
//		}
//
//		next_game_tick += SKIP_TICKS;
//		sleep_time = next_game_tick - GetTickCount();
//		if (sleep_time >= 0)
//		{
//			//std::cout << "Sleep Time " << sleep_time  << std::endl;
//			Sleep(sleep_time);
//		}
//		fps++;
//		if (USE_IMAGE)
//		{
//			MARKERTRACKING_FLAG = false;
//			USE_IMAGE = false;
//		}
//	}
//}


//void Tracker::ExtendedKalmanFilter()
//{
//	bool logfileOn = false;
//
//}

void write_text_to_log_file(const std::string &text, const std::string &filename, bool &fileStarted)
{
	if (fileStarted)
	{
		std::ofstream log_file(
			filename, std::ios_base::out | std::ios_base::app);
		log_file << text << std::endl;
	}
	else
	{
		std::ofstream log_file(
			filename, std::ios_base::out);
		log_file << text << std::endl;
		fileStarted = true;
	}

}

//void Tracker::ExtendedKalmanFilter1()
//{
//	
//	//TESTING VARS INITIALIZATION---------
//	/*bool debugInput = false;
//	int debugInputVar = 0;
//	client->DebugInput(0);*/
//	string consoleTitle = "Client " + std::to_string(client->getID()) + " Logging Console";
//	LoggingConsole.Create(consoleTitle.c_str());
//	stringstream stringStream;
//	float realAccel[3] = { 0, 0, 0 }; //records the unfiltered acceleration for comparison purpouses
//
//	//------------------------------------
//
//	bool LOG_FILE_DEBUG = true;
//	if (LOG_FILE_DEBUG)
//		logFiles.insert(std::pair<std::string, boolean>("log_file.txt", false));
//
//	float lowPassFilterVar = 0.3; // Low Pass filter in case it is used (set 1 to turn off).
//
//	cv::KalmanFilter KFo = cv::KalmanFilter(1, 1, 1);
//	cv::Mat Ao = cv::Mat_<float>(1, 1);
//	cv::Mat Bo = cv::Mat_<float>(1, 1);
//	cv::Mat Uo = cv::Mat_<float>(1, 1);
//	cv::Mat Zo = cv::Mat_<float>(1, 1);
//	cv::Mat Ho = cv::Mat_<float>(1, 1);
//	cv::Mat estimatedo = cv::Mat_<float>(1, 1);
//	estimatedo.at<float>(0, 0) = 0;
//	KFo.statePost.at<float>(0, 0) = 0;
//	float lastAngleInRads = 0;
//	float gyroError = 0.03;							//Gyro sensor measurment error
//
//	cv::KalmanFilter KFx = cv::KalmanFilter(2, 2, 1); // [x, vx] [1marker, wifi, (sensors?)] [ax(accelerometer)]
//	cv::Mat Ax = cv::Mat_<float>(2, 2);
//	cv::Mat Bx = cv::Mat_<float>(2, 1);
//	cv::Mat Ux = cv::Mat_<float>(1, 1);
//	cv::Mat Zx = cv::Mat_<float>(2, 1);
//	cv::Mat Hx = cv::Mat_<float>(2, 2);
//	float prevXAccel = 0;							//keeping track of the previous Xaccelaration
//	float accelError = 0.00022;//0.00022;						//Accelerator sensor Measurment Error
//
//
//
//	cv::KalmanFilter KFy = cv::KalmanFilter(2, 2, 1);
//	cv::Mat Ay = cv::Mat_<float>(2, 2);
//	cv::Mat By = cv::Mat_<float>(2, 1);
//	cv::Mat Uy = cv::Mat_<float>(1, 1);
//	cv::Mat Zy = cv::Mat_<float>(2, 1);
//	cv::Mat Hy = cv::Mat_<float>(2, 2);
//	float prevYAccel = 0;
//
//	bool USE_IMAGE = false;
//
//	double lastGyroTS = -1;
//	double gyroTS = 0;
//	double lastAccelTS = - 1;
//	double accelTS = 0;
//
//	const int ITERS_PER_SECOND = KFIPS;
//	const int SKIP_TICKS = 1000 / ITERS_PER_SECOND;
//
//	DWORD next_iter_tick = GetTickCount();
//	int sleep_time = 0;
//	bool kalman_is_running = true;
//	DWORD startCount = GetTickCount();
//	int ips = 0;
//
//	float accel[3] = { 0, 0, 0 };
//	float lastValidAccel[3] = { 0, 0, 0 };
//	float accelErrorWindow =0.15;
//
//	float gyro[3] = { 0, 0, 0 };
//	float lastValidGyro[3] = { 0, 0, 0 };
//	float orientation[3] = { 0, 0, 0 };
//	float gyroErrorWindow = 0.1;
//
//	int recalibrationCounterX = 0;
//	int xTries = 3;
//	int recalibrationCounterY = 0;
//	int yTries = 3;
//	vector<float> marker = {0,0,0};
//
//	while (!EXIT_FLAG)
//	{
//
//		//if (debugInput && debugInput < 10)
//		//{
//		//	client->DebugInput(debugInputVar);
//		//	debugInputVar++;
//		//}
//
//		if (GetTickCount() - startCount > 1000) //Counts the iterations per second
//		{
//			this->KalmanPS = ips;
//			startCount = GetTickCount();
//			ips = 0;
//		}
//		#pragma region Variables Update
//		if (lastGyroTS == -1 && client->gyroTS > 1)
//			lastGyroTS = client->gyroTS - 1;
//
//		gyroTS = client->gyroTS;
//		gyro[0] = client->gyro[0];
//		gyro[1] = client->gyro[1];
//		gyro[2] = client->gyro[2];
//
//		if (lastAccelTS == -1 && client->gyroTS > 0)
//			lastAccelTS = client->accelTS - 1;
//
//		accelTS = client->accelTS;
//		accel[0] = client->accel[0];
//		accel[1] = client->accel[1];
//		accel[2] = client->accel[2];
//
//		orientation[0] = client->orientation[0]; //x
//		orientation[1] = client->orientation[2]; //y 
//		orientation[2] = client->orientation[1]; //z 
//
//		double gyrodeltaTime = ((gyroTS - lastGyroTS) / 1000000000.0f);
//		double acceldeltaTime = ((accelTS - lastAccelTS) / 1000000000.0f);
//		#pragma endregion
//		
//		#pragma region Gyro Variable Filter
//		if (gyrodeltaTime > 0)
//		{
//			if (abs(lastValidGyro[0] - gyro[0]) <= gyroErrorWindow)
//			{
//				gyro[0] = 0;
//			}
//			else
//			{
//				lastValidGyro[0] = gyro[0];
//			}
//			
//			if (abs(lastValidGyro[1] - gyro[1]) <= gyroErrorWindow)
//			{
//				gyro[1] = 0;
//			}
//			else
//			{
//				lastValidGyro[1] = gyro[1];
//			}
//
//			if (abs(lastValidGyro[2] - gyro[2]) <= gyroErrorWindow)
//			{
//				gyro[2] = 0;
//			}
//			else
//			{
//				lastValidGyro[2] = gyro[2];
//			}
//		}
//		#pragma endregion
//		
//		#pragma region Acceleration Variable Filter
//		float tempAccel[3] = { 0, 0, 0 };
//
//		tempAccel[0] = accel[0] * cos(orientation[1])* cos(orientation[2]) + accel[1] * cos(orientation[1])*sin(orientation[2]) + accel[2] * sin(orientation[1]);
//		tempAccel[1] = accel[0] * -sin(orientation[2]) + accel[1] * cos(orientation[2]);
//		tempAccel[2] = accel[0] * sin(orientation[1])* cos(orientation[2]) + accel[1] * sin(orientation[1])*sin(orientation[2]) + accel[2] * cos(orientation[1]);
//
//		//has no further affect in math...its just for graph compairson
//		realAccel[0] = tempAccel[0];
//		realAccel[1] = tempAccel[1];
//		realAccel[2] = tempAccel[2];
//		//---------------------------------------------------------------
//
//
//		if (acceldeltaTime > 0)
//		{
//			if (abs(lastValidAccel[0] - tempAccel[0]) <= accelErrorWindow)
//			{
//				accel[0] = lastValidAccel[0];
//			}
//			else
//			{
//				accel[0] = tempAccel[0];
//				lastValidAccel[0] = accel[0];
//			}
//
//			if (abs(lastValidAccel[1] - tempAccel[1]) <= accelErrorWindow)
//			{
//				accel[1] = lastValidAccel[1];
//				recalibrationCounterX++;
//			}
//			else
//			{
//				accel[1] = tempAccel[1];
//				lastValidAccel[1] = accel[1];
//				recalibrationCounterX = 0;
//			}
//
//			if (abs(lastValidAccel[2] - tempAccel[2]) <= accelErrorWindow)
//			{
//				accel[2] = lastValidAccel[2];
//				recalibrationCounterY++;
//			}
//			else
//			{
//				accel[2] = tempAccel[2];
//				lastValidAccel[2] = accel[2];
//				recalibrationCounterY = 0;
//			}
//		}
//		#pragma endregion
//		
//		if (MARKERTRACKING_FLAG)
//		{
//			marker = MarkerPosition;
//			USE_IMAGE = true;
//		}
//			
//
//		// TESTING BLOCK -----------------------------
//		//if (USE_IMAGE)
//		//{
//
//		//	cv::Mat prediction = KF.predict();
//		//	cv::Point predictPt(prediction.at<float>(0), prediction.at<float>(1));
//
//		//	measurments(0) = MarkerPosition[0];
//		//	measurments(1) = MarkerPosition[1];
//
//		//	cv::Point measPt(measurments(0), measurments(1));
//		//	posv.push_back(measPt);
//
//		//	cv::Mat estimated = KF.correct(measurments);
//		//	cv::Point statePt(estimated.at<float>(0), estimated.at<float>(1));
//
//		//	kalmanv.push_back(statePt);
//
//		//	/*client->kalmanPos[0] = estimated.at<float>(0);
//		//	client->kalmanPos[1] = estimated.at<float>(1);
//		//	client->kalmanPos[2] = MarkerPosition[2];*/
//		//}
//		//--------------------------------------------
//
//		#pragma region Theta Kalman Filter (Angle)
//		if (gyrodeltaTime > 0 || MARKERTRACKING_FLAG)
//		{
//			float PreviousAngle = client->kalmanPos[2];// KFo.statePost.at<float>(0, 0) * 180 / CV_PI;
//			int TimesAround = PreviousAngle / 360;
//			float MarkerAngle = marker[2];
//			float nearAngle = MarkerAngle + TimesAround * 360;
//			float correctedMarkerAngle = 0;
//			if (abs(PreviousAngle - (nearAngle + 360)) <= abs(PreviousAngle - nearAngle))
//			{
//				correctedMarkerAngle = nearAngle + 360;
//			}
//			else
//			{
//				if (abs(PreviousAngle - (nearAngle - 360)) <= abs(PreviousAngle - nearAngle))
//					correctedMarkerAngle = nearAngle - 360;
//				else
//					correctedMarkerAngle = nearAngle;
//			}
//
//			Ao.at<float>(0, 0) = 1;
//			Bo.at<float>(0, 0) = gyrodeltaTime;
//			Zo.at<float>(0, 0) = correctedMarkerAngle * CV_PI / 180;
//			Uo.at<float>(0, 0) = gyro[0]; // rads/s
//
//			if (USE_IMAGE)
//				Ho.at<float>(0, 0) = 1;
//			else
//				Ho.at<float>(0, 0) = 0;
//
//			KFo.transitionMatrix = Ao;
//			KFo.controlMatrix = Bo;
//			KFo.measurementMatrix = Ho;
//
//			cv::setIdentity(KFo.processNoiseCov, cv::Scalar(gyroError*CV_PI / 180)); //Q
//			cv::setIdentity(KFo.measurementNoiseCov, cv::Scalar(0.1)); //R
//
//			cv::Mat predictiono = KFo.predict(Uo);
//			cv::Mat estimatedo = KFo.correct(Zo);
//
//			//cout << "Xo^ =" << KFo.statePre << endl;
//			//cout << "Xo^+1 =" << KFo.statePost << endl;
//			//cout << "K = " << KFo.gain << endl << endl;;
//
//			float radToDegrees = estimatedo.at<float>(0, 0) * 180 / CV_PI;
//			client->kalmanPos[2] = fmod(radToDegrees, 360);
//			lastGyroTS = gyroTS;
//		}
//		#pragma endregion
//
//		if (acceldeltaTime > 0 || MARKERTRACKING_FLAG)
//		{
//			#pragma region X Kalman Filter
//			
//			//Ax =	| 1  delta(Time)|
//			//		| 0		1		|	
//			Ax.at<float>(0, 0) = 1;
//			Ax.at<float>(0, 1) = acceldeltaTime;
//			Ax.at<float>(1, 0) = 0;
//
//			float tempaccelx = accel[1] * cos(client->kalmanPos[2] * CV_PI / 180) - accel[2] * sin(client->kalmanPos[2] * CV_PI / 180);
//			if (recalibrationCounterX > xTries) //if the device is stopped (no acceleration) then the speed is reset to 0
//			{
//				Ax.at<float>(1, 1) = 0;
//				if (tempaccelx < 0.05)
//					tempaccelx = 0;
//			}
//				
//			else
//				Ax.at<float>(1, 1) = 1;
//
//			Bx.at<float>(0, 0) = 0.5*(acceldeltaTime*acceldeltaTime);
//			Bx.at<float>(1, 0) = acceldeltaTime;
//
//			Zx.at<float>(0, 0) = marker[0];
//			Zx.at<float>(1, 0) = 0;
//
//			//Ux.at<float>(0, 0) = -accel[1] * 100;
//
//			//Ux.at<float>(0, 0) = (-accel[1] * cos(estimatedo.at<float>(0, 0)) + accel[2] * sin(estimatedo.at<float>(0, 0))) * 100;
//			//float tempaccelx = accel[1] * cos(client->kalmanPos[2] * CV_PI / 180) - accel[2] * sin(client->kalmanPos[2] * CV_PI / 180);
//			prevXAccel = prevXAccel*(1 - lowPassFilterVar) + (tempaccelx)*lowPassFilterVar;
//			Ux.at<float>(0, 0) = -prevXAccel * 100;
//
//			if (USE_IMAGE)
//				Hx.at<float>(0, 0) = 1;
//			else
//				Hx.at<float>(0, 0) = 0;
//			Hx.at<float>(0, 1) = 0;
//			Hx.at<float>(1, 0) = 0;
//			Hx.at<float>(1, 1) = 0;
//
//			KFx.transitionMatrix = Ax;
//			KFx.controlMatrix = Bx;
//			KFx.measurementMatrix = Hx;
//
//			cv::setIdentity(KFx.processNoiseCov, cv::Scalar(accelError)); //Q
//			cv::setIdentity(KFx.measurementNoiseCov, cv::Scalar(1));
//
//			cv::Mat predictionx = KFx.predict(Ux);
//			cv::Mat estimatedx = KFx.correct(Zx);
//
//
//
//			#pragma endregion
//
//			#pragma region Y Kalman Filter
//			Ay.at<float>(0, 0) = 1;
//			Ay.at<float>(0, 1) = acceldeltaTime;
//			Ay.at<float>(1, 0) = 0;
//
//			float tempaccely = accel[1] * sin(client->kalmanPos[2] * CV_PI / 180) + accel[2] * cos(client->kalmanPos[2] * CV_PI / 180);
//			if (recalibrationCounterY > yTries)
//			{
//				Ay.at<float>(1, 1) = 0;
//				if (tempaccely < 0.05)
//					tempaccely = 0;
//			}
//			else
//				Ay.at<float>(1, 1) = 1;
//
//			By.at<float>(0, 0) = 0.5*(acceldeltaTime*acceldeltaTime);
//			By.at<float>(1, 0) = acceldeltaTime;
//
//			Zy.at<float>(0, 0) = marker[1];
//			Zy.at<float>(1, 0) = 0;// ver como é com o beacon
//
//			//Uy.at<float>(0, 0) = accel[2] * 100;
//
//			//Uy.at<float>(0, 0) = (accel[2] * cos(estimatedo.at<float>(0, 0)) - (-accel[1])*sin(estimatedo.at<float>(0, 0))) * 100;
//			
//			prevYAccel = prevYAccel*(1 - lowPassFilterVar) + (tempaccely)*lowPassFilterVar;
//			Uy.at<float>(0, 0) = prevYAccel * 100;
//
//			if (USE_IMAGE)
//				Hy.at<float>(0, 0) = 1;
//			else
//				Hy.at<float>(0, 0) = 0;
//			Hy.at<float>(0, 1) = 0;
//			Hy.at<float>(1, 0) = 0; // Resolve this to the beacon
//			Hy.at<float>(1, 1) = 0;
//
//			KFy.transitionMatrix = Ay;
//			KFy.controlMatrix = By;
//			KFy.measurementMatrix = Hy;
//
//			cv::setIdentity(KFy.processNoiseCov, cv::Scalar(accelError)); //Q
//			cv::setIdentity(KFy.measurementNoiseCov, cv::Scalar(1));
//
//			cv::Mat predictiony = KFy.predict(Uy);
//			cv::Mat estimatedy = KFy.correct(Zy);
//			#pragma endregion
//
//			//RETREIVE RESULTS--------------
//			client->kalmanPos[0] = estimatedx.at<float>(0, 0);
//			client->kalmanPos[1] = estimatedy.at<float>(0, 0);
//			lastAccelTS = accelTS;
//
//			if (LOG_FILE_DEBUG)
//			{
//				string logText("" + to_string(acceldeltaTime) + " "
//					+ to_string(accel[0]) + " " + to_string(Ux.at<float>(0, 0)) + " " + to_string(Uy.at<float>(0, 0)) + " "
//					+ to_string(KFx.statePost.at<float>(1, 0)) + " " + to_string(KFy.statePost.at<float>(1, 0)) + " "
//					+ to_string(KFx.statePost.at<float>(0, 0)) + " " + to_string(KFy.statePost.at<float>(0, 0)) + " " + to_string(KFx.statePre.at<float>(0, 0)) + " "
//					+ to_string(KFy.statePre.at<float>(0, 0)) + " " + to_string(MARKERTRACKING_FLAG ? 1 : 0)); //ts ax ay az vx vy x y x' y' marker
//
//				//string logText("" + to_string(acceldeltaTime) + " "
//				//	+ to_string(KFx.statePre.at<float>(0, 0)) + " " + to_string(KFy.statePre.at<float>(0, 0)) + " " + to_string(USE_IMAGE ? 1 : 0)); //ts x' y'
//				auto iter = logFiles.find("log_file.txt");
//				write_text_to_log_file(logText, iter->first, (iter->second));
//
//				//TESTING BLOCK --------------------------------
//				//string logText2("" + to_string(acceldeltaTime) + " " + to_string(KF.statePre.at<float>(0, 0)) + " " + to_string(KF.statePre.at<float>(1, 0))); //ts x' y'
//				//auto iter2 = logFiles.find("marker_log_file.txt");
//				//write_text_to_log_file(logText2, iter2->first, (iter2->second));
//				//----------------------------------------------
//			}
//
//			stringStream.str("");
//			stringStream
//				<< "-----------VARIABLES------------" << endl
//				<< "Ax =\n" << KFx.transitionMatrix << endl
//				<< "Bx =\n" << KFx.controlMatrix << endl
//				<< "Ux =\n" << Ux << endl
//				<< "Zx =\n" << Zx << endl
//				<< "Qx =\n" << KFx.processNoiseCov << endl
//				<< "Rx =\n" << KFx.measurementNoiseCov << endl
//				<< "Hx =\n" << Hx << endl
//
//				<< "-----------PREDICTION-----------" << endl
//				<< "Xx^ =\n" << KFx.statePre << endl
//				<< "Px^ =\n" << KFx.errorCovPre << endl
//
//				<< "-------------UPDATE-------------" << endl
//				<< "Xx^+1 =\n" << KFx.statePost << endl
//				<< "K =\n " << KFx.gain << endl << endl
//				<< "Px^+1 =\n" << KFx.errorCovPost << endl;
//
//			LoggingConsole.cls();
//			LoggingConsole.printf(stringStream.str().c_str());
//		}
//
//
//		next_iter_tick += SKIP_TICKS;
//		sleep_time = next_iter_tick - GetTickCount();
//		if (sleep_time >= 0)
//		{
//			//std::cout << "Sleep Time " << sleep_time  << std::endl;
//			Sleep(sleep_time);
//		}
//		ips++;
//		if (USE_IMAGE)
//		{
//			MARKERTRACKING_FLAG = false;
//			USE_IMAGE = false;
//		}
//
//	}
//}

void Tracker::ExtendedKalmanFilter2()
{

	//TESTING VARS INITIALIZATION---------
	/*bool debugInput = false;
	int debugInputVar = 0;
	client->DebugInput(0);*/
	string consoleTitle = "Client " + std::to_string(client->getID()) + " Logging Console";
	LoggingConsole.Create(consoleTitle.c_str());
	stringstream stringStream;
	float realAccel[3] = { 0, 0, 0 }; //records the unfiltered acceleration for comparison purpouses

	//------------------------------------

	bool LOG_FILE_DEBUG = true;
	if (LOG_FILE_DEBUG)
		logFiles.insert(std::pair<std::string, boolean>("log_file.txt", false));

	float lowPassFilterVar = 0.3; // Low Pass filter in case it is used (set 1 to turn off).

	cv::KalmanFilter KFo = cv::KalmanFilter(2, 2);
	cv::Mat Ao = cv::Mat_<float>(2, 2);
	cv::Mat Zo = cv::Mat_<float>(2, 1);
	cv::Mat Ho = cv::Mat_<float>(2, 2);
	cv::Mat estimatedo = cv::Mat_<float>(1, 1);
	estimatedo.at<float>(0, 0) = 0;
	KFo.statePost.at<float>(0, 0) = 0;
	float lastAngleInRads = 0;
	float gyroError = 0.03;							//Gyro sensor measurment error

	cv::KalmanFilter KFx = cv::KalmanFilter(3, 3); // [x, vx] [1marker, wifi, (sensors?)] [ax(accelerometer)]
	cv::Mat Ax = cv::Mat_<float>(3, 3);
	cv::Mat Zx = cv::Mat_<float>(3, 1);
	cv::Mat Hx = cv::Mat_<float>(3, 3);
	float prevXAccel = 0;							//keeping track of the previous Xaccelaration
	float accelError = 0.00022;//0.00022;						//Accelerator sensor Measurment Error



	cv::KalmanFilter KFy = cv::KalmanFilter(3, 3);
	cv::Mat Ay = cv::Mat_<float>(3, 3);
	cv::Mat Zy = cv::Mat_<float>(3, 1);
	cv::Mat Hy = cv::Mat_<float>(3, 3);
	float prevYAccel = 0;

	bool USE_IMAGE = false;

	double lastGyroTS = -1;
	double gyroTS = 0;
	double lastAccelTS = -1;
	double accelTS = 0;

	const int ITERS_PER_SECOND = KFIPS;
	const int SKIP_TICKS = 1000 / ITERS_PER_SECOND;

	DWORD next_iter_tick = GetTickCount();
	int sleep_time = 0;
	bool kalman_is_running = true;
	DWORD startCount = GetTickCount();
	int ips = 0;

	float accel[3] = { 0, 0, 0 };
	float lastValidAccel[3] = { 0, 0, 0 };
	float accelErrorWindow = 0.15;

	float gyro[3] = { 0, 0, 0 };
	float lastValidGyro[3] = { 0, 0, 0 };
	float orientation[3] = { 0, 0, 0 };
	float gyroErrorWindow = 0.1;

	int recalibrationCounterX = 0;
	int xTries = 3;
	int recalibrationCounterY = 0;
	int yTries = 3;
	vector<float> marker = { 0, 0, 0 };

	while (!EXIT_FLAG)
	{

		//if (debugInput && debugInput < 10)
		//{
		//	client->DebugInput(debugInputVar);
		//	debugInputVar++;
		//}

		if (GetTickCount() - startCount > 1000) //Counts the iterations per second
		{
			this->KalmanPS = ips;
			startCount = GetTickCount();
			ips = 0;
		}
#pragma region Variables Update
		if (lastGyroTS == -1 && client->gyroTS > 1)
			lastGyroTS = client->gyroTS - 1;

		gyroTS = client->gyroTS;
		gyro[0] = client->gyro[0];
		gyro[1] = client->gyro[1];
		gyro[2] = client->gyro[2];

		if (lastAccelTS == -1 && client->gyroTS > 0)
			lastAccelTS = client->accelTS - 1;

		accelTS = client->accelTS;
		accel[0] = client->accel[0];
		accel[1] = client->accel[1];
		accel[2] = client->accel[2];

		orientation[0] = client->orientation[0]; //x
		orientation[1] = client->orientation[2]; //y 
		orientation[2] = client->orientation[1]; //z 

		double gyrodeltaTime = ((gyroTS - lastGyroTS) / 1000000000.0f);
		double acceldeltaTime = ((accelTS - lastAccelTS) / 1000000000.0f);
#pragma endregion

#pragma region Gyro Variable Filter
		if (gyrodeltaTime > 0)
		{
			if (abs(lastValidGyro[0] - gyro[0]) <= gyroErrorWindow)
			{
				gyro[0] = 0;
			}
			else
			{
				lastValidGyro[0] = gyro[0];
			}

			if (abs(lastValidGyro[1] - gyro[1]) <= gyroErrorWindow)
			{
				gyro[1] = 0;
			}
			else
			{
				lastValidGyro[1] = gyro[1];
			}

			if (abs(lastValidGyro[2] - gyro[2]) <= gyroErrorWindow)
			{
				gyro[2] = 0;
			}
			else
			{
				lastValidGyro[2] = gyro[2];
			}
		}
#pragma endregion

#pragma region Acceleration Variable Filter
		float tempAccel[3] = { 0, 0, 0 };

		tempAccel[0] = accel[0] * cos(orientation[1])* cos(orientation[2]) + accel[1] * cos(orientation[1])*sin(orientation[2]) + accel[2] * sin(orientation[1]);
		tempAccel[1] = accel[0] * -sin(orientation[2]) + accel[1] * cos(orientation[2]);
		tempAccel[2] = accel[0] * sin(orientation[1])* cos(orientation[2]) + accel[1] * sin(orientation[1])*sin(orientation[2]) + accel[2] * cos(orientation[1]);

		//has no further affect in math...its just for graph compairson
		realAccel[0] = tempAccel[0];
		realAccel[1] = tempAccel[1];
		realAccel[2] = tempAccel[2];
		//---------------------------------------------------------------


		if (acceldeltaTime > 0)
		{
			if (abs(lastValidAccel[0] - tempAccel[0]) <= accelErrorWindow)
			{
				accel[0] = lastValidAccel[0];
			}
			else
			{
				accel[0] = tempAccel[0];
				lastValidAccel[0] = accel[0];
			}

			if (abs(lastValidAccel[1] - tempAccel[1]) <= accelErrorWindow)
			{
				accel[1] = lastValidAccel[1];
				recalibrationCounterX++;
			}
			else
			{
				accel[1] = tempAccel[1];
				lastValidAccel[1] = accel[1];
				recalibrationCounterX = 0;
			}

			if (abs(lastValidAccel[2] - tempAccel[2]) <= accelErrorWindow)
			{
				accel[2] = lastValidAccel[2];
				recalibrationCounterY++;
			}
			else
			{
				accel[2] = tempAccel[2];
				lastValidAccel[2] = accel[2];
				recalibrationCounterY = 0;
			}
		}
#pragma endregion

		if (MARKERTRACKING_FLAG && (marker[0] != MarkerPosition[0] || marker[1] != MarkerPosition[1] || marker[2] != MarkerPosition[2]))
		{
			marker = MarkerPosition;
			USE_IMAGE = true;
			MARKERTRACKING_FLAG = false;
		}


		// TESTING BLOCK -----------------------------
		//if (USE_IMAGE)
		//{

		//	cv::Mat prediction = KF.predict();
		//	cv::Point predictPt(prediction.at<float>(0), prediction.at<float>(1));

		//	measurments(0) = MarkerPosition[0];
		//	measurments(1) = MarkerPosition[1];

		//	cv::Point measPt(measurments(0), measurments(1));
		//	posv.push_back(measPt);

		//	cv::Mat estimated = KF.correct(measurments);
		//	cv::Point statePt(estimated.at<float>(0), estimated.at<float>(1));

		//	kalmanv.push_back(statePt);

		//	/*client->kalmanPos[0] = estimated.at<float>(0);
		//	client->kalmanPos[1] = estimated.at<float>(1);
		//	client->kalmanPos[2] = MarkerPosition[2];*/
		//}
		//--------------------------------------------

#pragma region Theta Kalman Filter (Angle)
		if (gyrodeltaTime > 0 || USE_IMAGE)
		{
			float PreviousAngle = client->kalmanPos[2];// KFo.statePost.at<float>(0, 0) * 180 / CV_PI;
			int TimesAround = PreviousAngle / 360;
			float MarkerAngle = marker[2];
			float nearAngle = MarkerAngle + TimesAround * 360;
			float correctedMarkerAngle = 0;
			if (abs(PreviousAngle - (nearAngle + 360)) <= abs(PreviousAngle - nearAngle))
			{
				correctedMarkerAngle = nearAngle + 360;
			}
			else
			{
				if (abs(PreviousAngle - (nearAngle - 360)) <= abs(PreviousAngle - nearAngle))
					correctedMarkerAngle = nearAngle - 360;
				else
					correctedMarkerAngle = nearAngle;
			}


			Ao.at<float>(0, 0) = 1;
			Ao.at<float>(0, 1) = gyrodeltaTime;
			Ao.at<float>(1, 0) = 0;
			Ao.at<float>(1, 1) = 1;

			Zo.at<float>(0, 0) = correctedMarkerAngle * CV_PI / 180;
			Zo.at<float>(1, 0) = gyro[0]; // rads/s

			if (USE_IMAGE)
				Ho.at<float>(0, 0) = 1;
			else
				Ho.at<float>(0, 0) = 0;
			Ho.at<float>(0, 1) = 0;
			Ho.at<float>(1, 0) = 0;
			Ho.at<float>(1, 1) = 1;

			KFo.transitionMatrix = Ao;
			KFo.measurementMatrix = Ho;

			cv::setIdentity(KFo.processNoiseCov, cv::Scalar(gyroError*CV_PI / 180)); //Q
			cv::setIdentity(KFo.measurementNoiseCov, cv::Scalar(0.1)); //R

			cv::Mat predictiono = KFo.predict();
			cv::Mat estimatedo = KFo.correct(Zo);

			//cout << "Xo^ =" << KFo.statePre << endl;
			//cout << "Xo^+1 =" << KFo.statePost << endl;
			//cout << "K = " << KFo.gain << endl << endl;;

			float radToDegrees = estimatedo.at<float>(0, 0) * 180 / CV_PI;
			client->kalmanPos[2] = fmod(radToDegrees, 360);
			lastGyroTS = gyroTS;
		}
#pragma endregion

		if (acceldeltaTime > 0 || USE_IMAGE)
		{
#pragma region X Kalman Filter

			//Ax =	| 1		delta(Time)  delta(time)^2   |
			//		| 0			1				1		 |
			//		| 0			0				1		 |
			Ax.at<float>(0, 0) = 1;
			Ax.at<float>(0, 1) = acceldeltaTime;
			Ax.at<float>(0, 2) = 0.5*(acceldeltaTime*acceldeltaTime);
			Ax.at<float>(1, 0) = 0;
			Ax.at<float>(1, 1) = 1;
			Ax.at<float>(1, 2) = acceldeltaTime;
			Ax.at<float>(2, 0) = 0;
			Ax.at<float>(2, 1) = 0;

			float tempaccelx = accel[1] * cos(client->kalmanPos[2] * CV_PI / 180) - accel[2] * sin(client->kalmanPos[2] * CV_PI / 180);
			if (recalibrationCounterX > xTries) //if the device is stopped (no acceleration) then the speed is reset to 0
			{
				Ax.at<float>(2, 2) = 0;
				if (tempaccelx < 0.05)
					tempaccelx = 0;
			}

			else
				Ax.at<float>(2, 2) = 1;

			Zx.at<float>(0, 0) = marker[0];
			prevXAccel = prevXAccel*(1 - lowPassFilterVar) + (tempaccelx)*lowPassFilterVar;
			Zx.at<float>(1, 0) = -prevXAccel * 100;
			Zx.at<float>(2, 0) = 0;

			//Ux.at<float>(0, 0) = -accel[1] * 100;

			//Ux.at<float>(0, 0) = (-accel[1] * cos(estimatedo.at<float>(0, 0)) + accel[2] * sin(estimatedo.at<float>(0, 0))) * 100;
			//float tempaccelx = accel[1] * cos(client->kalmanPos[2] * CV_PI / 180) - accel[2] * sin(client->kalmanPos[2] * CV_PI / 180);

			Hx.zeros(3, 3, CV_32F);

			if (USE_IMAGE)
				Hx.at<float>(0, 0) = 1;
			else
				Hx.at<float>(0, 0) = 0;

			Hx.at<float>(0, 1) = 0;
			Hx.at<float>(0, 2) = 0;

			Hx.at<float>(1, 0) = 0;
			Hx.at<float>(1, 1) = 0;
			Hx.at<float>(1, 2) = 1; //accel;

			
			Hx.at<float>(2, 0) = 0; // wifi
			Hx.at<float>(2, 1) = 0;
			Hx.at<float>(2, 2) = 0;

			KFx.transitionMatrix = Ax;
			KFx.measurementMatrix = Hx;

			cv::setIdentity(KFx.processNoiseCov, cv::Scalar(accelError)); //Q
			cv::setIdentity(KFx.measurementNoiseCov, cv::Scalar(0.1));

			cv::Mat predictionx = KFx.predict();
			cv::Mat estimatedx = KFx.correct(Zx);



#pragma endregion

#pragma region Y Kalman Filter
			//Ay =	| 1		delta(Time)  delta(time)^2   |
			//		| 0			1				1		 |
			//		| 0			0				1		 |
			Ay.at<float>(0, 0) = 1;
			Ay.at<float>(0, 1) = acceldeltaTime;
			Ay.at<float>(0, 2) = 0.5*(acceldeltaTime*acceldeltaTime);
			Ay.at<float>(1, 0) = 0;
			Ay.at<float>(1, 1) = 1;
			Ay.at<float>(1, 2) = acceldeltaTime;
			Ay.at<float>(2, 0) = 0;
			Ay.at<float>(2, 1) = 0;

			float tempaccely = accel[1] * sin(client->kalmanPos[2] * CV_PI / 180) + accel[2] * cos(client->kalmanPos[2] * CV_PI / 180);
			if (recalibrationCounterY > yTries)
			{
				Ay.at<float>(2, 2) = 0;
				if (tempaccely < 0.05)
					tempaccely = 0;
			}
			else
				Ay.at<float>(2, 2) = 1;


			Zy.at<float>(0, 0) = marker[1];
			prevYAccel = prevYAccel*(1 - lowPassFilterVar) + (tempaccely)*lowPassFilterVar;
			Zy.at<float>(1, 0) = prevYAccel * 100;
			Zy.at<float>(2, 0) = 0;// ver como é com o beacon

			//Uy.at<float>(0, 0) = accel[2] * 100;

			//Uy.at<float>(0, 0) = (accel[2] * cos(estimatedo.at<float>(0, 0)) - (-accel[1])*sin(estimatedo.at<float>(0, 0))) * 100;

			Hy.zeros(3, 3, CV_32F);

			if (USE_IMAGE)
				Hy.at<float>(0, 0) = 1;
			else
				Hy.at<float>(0, 0) = 0;

			Hy.at<float>(0, 1) = 0;
			Hy.at<float>(0, 2) = 0;

			Hy.at<float>(1, 0) = 0;
			Hy.at<float>(1, 1) = 0;
			Hy.at<float>(1, 2) = 1; //accel;


			Hy.at<float>(2, 0) = 0; // wifi
			Hy.at<float>(2, 1) = 0;
			Hy.at<float>(2, 2) = 0;

			KFy.transitionMatrix = Ay;
			KFy.measurementMatrix = Hy;

			cv::setIdentity(KFy.processNoiseCov, cv::Scalar(accelError)); //Q
			cv::setIdentity(KFy.measurementNoiseCov, cv::Scalar(0.1));

			cv::Mat predictiony = KFy.predict();
			cv::Mat estimatedy = KFy.correct(Zy);
#pragma endregion

			//RETREIVE RESULTS--------------
			client->kalmanPos[0] = estimatedx.at<float>(0, 0);
			client->kalmanPos[1] = estimatedy.at<float>(0, 0);
			lastAccelTS = accelTS;

			if (LOG_FILE_DEBUG)
			{
				string logText("" + to_string(acceldeltaTime) + " "
					+ to_string(accel[0]) + " " + to_string(Zx.at<float>(1, 0)) + " " + to_string(Zy.at<float>(1, 0)) + " "
					+ to_string(KFx.statePost.at<float>(1, 0)) + " " + to_string(KFy.statePost.at<float>(1, 0)) + " "
					+ to_string(KFx.statePost.at<float>(0, 0)) + " " + to_string(KFy.statePost.at<float>(0, 0)) + " " + to_string(KFx.statePre.at<float>(0, 0)) + " "
					+ to_string(KFy.statePre.at<float>(0, 0)) + " " + to_string(MARKERTRACKING_FLAG ? 1 : 0)); //ts ax ay az vx vy x y x' y' marker

				//string logText("" + to_string(acceldeltaTime) + " "
				//	+ to_string(KFx.statePre.at<float>(0, 0)) + " " + to_string(KFy.statePre.at<float>(0, 0)) + " " + to_string(USE_IMAGE ? 1 : 0)); //ts x' y'
				auto iter = logFiles.find("log_file.txt");
				write_text_to_log_file(logText, iter->first, (iter->second));

				//TESTING BLOCK --------------------------------
				//string logText2("" + to_string(acceldeltaTime) + " " + to_string(KF.statePre.at<float>(0, 0)) + " " + to_string(KF.statePre.at<float>(1, 0))); //ts x' y'
				//auto iter2 = logFiles.find("marker_log_file.txt");
				//write_text_to_log_file(logText2, iter2->first, (iter2->second));
				//----------------------------------------------
			}

			stringStream.str("");
			stringStream
				<< "-----------VARIABLES------------" << endl
				<< "Ax =\n" << KFx.transitionMatrix << endl
				//<< "Bx =\n" << KFx.controlMatrix << endl
				//<< "Ux =\n" << Ux << endl
				<< "Zx =\n" << Zx << endl
				<< "Qx =\n" << KFx.processNoiseCov << endl
				<< "Rx =\n" << KFx.measurementNoiseCov << endl
				<< "Hx =\n" << KFx.measurementMatrix << endl

				<< "-----------PREDICTION-----------" << endl
				<< "Xx^ =\n" << KFx.statePre << endl
				<< "Px^ =\n" << KFx.errorCovPre << endl

				<< "-------------UPDATE-------------" << endl
				<< "Xx^+1 =\n" << KFx.statePost << endl
				<< "K =\n " << KFx.gain << endl << endl
				<< "Px^+1 =\n" << KFx.errorCovPost << endl;

			LoggingConsole.cls();
			LoggingConsole.printf(stringStream.str().c_str());
		}


		next_iter_tick += SKIP_TICKS;
		sleep_time = next_iter_tick - GetTickCount();
		if (sleep_time >= 0)
		{
			//std::cout << "Sleep Time " << sleep_time  << std::endl;
			Sleep(sleep_time);
		}
		ips++;
		if (USE_IMAGE)
		{
			
			USE_IMAGE = false;
		}

	}
}

void Tracker::ExtendedKalmanFilter3()
{
	//TESTING VARS INITIALIZATION---------
	/*bool debugInput = false;
	int debugInputVar = 0;
	client->DebugInput(0);*/
	string consoleTitle = "Client " + std::to_string(client->getID()) + " Logging Console";
	LoggingConsole.Create(consoleTitle.c_str());
	stringstream stringStream;
	float realAccel[3] = { 0, 0, 0 }; //records the unfiltered acceleration in the device axis for comparison purpouses
	

	float rawAccel[3] = { 0, 0, 0 };
	float rawGyro[3] = { 0, 0, 0 };
	float rawIncl[3] = { 0, 0, 0 };

	//------------------------------------

	bool LOG_FILE_DEBUG = true;
	if (LOG_FILE_DEBUG)
		logFiles.insert(std::pair<std::string, boolean>("log_file.txt", false));

	float lowPassFilterVar = 1;// 0.3; // Low Pass filter in case it is used (set 1 to turn off).

	cv::KalmanFilter KFo = cv::KalmanFilter(2, 2);
	KFo.statePost.zeros(2, 1, CV_32F);
	KFo.statePre.zeros(3, 1, CV_32F);
	cv::Mat Ao = cv::Mat_<float>(2, 2);
	cv::Mat Zo = cv::Mat_<float>(2, 1);
	cv::Mat Ho = cv::Mat_<float>(2, 2);
	cv::Mat estimatedo = cv::Mat_<float>(1, 1);
	estimatedo.at<float>(0, 0) = 0;
	KFo.statePost.at<float>(0, 0) = 0;
	float lastAngleInRads = 0;
	float gyroError = 0.03;							//Gyro sensor measurment error

	cv::KalmanFilter KFx = cv::KalmanFilter(3, 3); // [x, vx] [1marker, wifi, (sensors?)] [ax(accelerometer)]
	KFx.statePost.zeros(3,1,CV_32F);
	KFx.statePre.zeros(3, 1, CV_32F);
	cv::Mat Ax = cv::Mat_<float>(3, 3);
	cv::Mat Zx = cv::Mat_<float>(3, 1);
	cv::Mat Hx = cv::Mat_<float>(3, 3);
	cv::Mat measurmentNoiseMatx = cv::Mat_<float>(3, 3);
	float prevXAccel = 0;							//keeping track of the previous Xaccelaration
	float accelError = 0.00022;//0.00022;						//Accelerator sensor Measurment Error
	float xProcessError = accelError;// 0.3113;


	cv::KalmanFilter KFy = cv::KalmanFilter(3, 3);
	KFy.statePost.zeros(3, 1, CV_32F);
	KFy.statePre.zeros(3, 1, CV_32F);
	cv::Mat Ay = cv::Mat_<float>(3, 3);
	cv::Mat Zy = cv::Mat_<float>(3, 1);
	cv::Mat Hy = cv::Mat_<float>(3, 3);
	cv::Mat measurmentNoiseMaty = cv::Mat_<float>(3, 3);
	float prevYAccel = 0;
	float yProcessError = accelError;// 0.8124;

	bool USE_IMAGE = false;

	double lastGyroTS = -1;
	double gyroTS = 0;
	double lastAccelTS = -1;
	double accelTS = 0;

	const int ITERS_PER_SECOND = KFIPS;
	const int SKIP_TICKS = 1000 / ITERS_PER_SECOND;

	DWORD next_iter_tick = GetTickCount();
	int sleep_time = 0;
	bool kalman_is_running = true;
	DWORD startCount = GetTickCount();
	int ips = 0;

	float accel[3] = { 0, 0, 0 };
	float lastValidAccel[3] = { 0, 0, 0 };
	float accelErrorWindow = 0.15;

	float gyro[3] = { 0, 0, 0 };
	float lastValidGyro[3] = { 0, 0, 0 };
	float orientation[3] = { 0, 0, 0 };
	float gyroErrorWindow = 0.1;

	int recalibrationCounterX = 0;
	int xTries = 3;
	int recalibrationCounterY = 0;
	int yTries = 3;
	vector<float> marker = { 0, 0, 0 };
	double sampleTime = 0;
	double runningAverage = 0;
	double samples = 0;


	while (!EXIT_FLAG)
	{
		sampleTime = GetTickCount();
		//if (debugInput && debugInput < 10)
		//{
		//	client->DebugInput(debugInputVar);
		//	debugInputVar++;
		//}

		if (GetTickCount() - startCount > 1000) //Counts the iterations per second
		{
			this->KalmanPS = ips;
			startCount = GetTickCount();
			ips = 0;
		}
#pragma region Variables Update
		if (lastGyroTS == -1 && client->gyroTS > 1)
			lastGyroTS = client->gyroTS - 1;

		gyroTS = client->gyroTS;
		rawGyro[0] = gyro[0] = client->gyro[0];
		rawGyro[1] = gyro[1] = client->gyro[1];
		rawGyro[2] = gyro[2] = client->gyro[2];

		if (lastAccelTS == -1 && client->gyroTS > 0)
			lastAccelTS = client->accelTS - 1;

		accelTS = client->accelTS;
		rawAccel[0] = accel[0] = client->accel[0];
		rawAccel[1] = accel[1] = client->accel[1];
		rawAccel[2] = accel[2] = client->accel[2];

		//Inclination in relation to the floor
		rawIncl[1] = orientation[0] = -client->orientation[1]; //X
		rawIncl[0] = orientation[1] = client->orientation[0]; //y
		rawIncl[2] = orientation[2] = client->orientation[2]; //z 

		double gyrodeltaTime = ((gyroTS - lastGyroTS) / 1000000000.0f);
		double acceldeltaTime = ((accelTS - lastAccelTS) / 1000000000.0f);
#pragma endregion

#pragma region Gyro Variable Filter
		if (gyrodeltaTime > 0)
		{
			if (abs(lastValidGyro[0] - gyro[0]) <= gyroErrorWindow)
			{
				gyro[0] = 0;
			}
			else
			{
				lastValidGyro[0] = gyro[0];
			}

			if (abs(lastValidGyro[1] - gyro[1]) <= gyroErrorWindow)
			{
				gyro[1] = 0;
			}
			else
			{
				lastValidGyro[1] = gyro[1];
			}

			if (abs(lastValidGyro[2] - gyro[2]) <= gyroErrorWindow)
			{
				gyro[2] = 0;
			}
			else
			{
				lastValidGyro[2] = gyro[2];
			}
		}
#pragma endregion

#pragma region Acceleration Variable Filter
		float tempAccel[3] = { 0, 0, 0 };

		/*tempAccel[0] = accel[0] * -sin(orientation[1]) + accel[2] * cos(orientation[1]);
		tempAccel[1] = accel[0] * cos(orientation[0])*cos(orientation[1]) + accel[1] * -sin(orientation[1]) + accel[2] * cos(orientation[0]) * sin(orientation[1]);
		tempAccel[2] = accel[0] * sin(orientation[0])*cos(orientation[2]) + accel[1] * cos(orientation[1]) + accel[2] * sin(orientation[0]) * sin(orientation[1]);*/
		tempAccel[0] = accel[0] * cos(orientation[1]) + accel[2] * sin(orientation[1]);
		tempAccel[1] = accel[0] * sin(orientation[0])*sin(orientation[1]) + accel[1] * cos(orientation[0]) + accel[2] * -sin(orientation[0]) * cos(orientation[1]);
		tempAccel[2] = accel[0] * cos(orientation[0])*sin(orientation[1]) + accel[1] * -sin(orientation[0]) + accel[2] * -cos(orientation[0]) * cos(orientation[1]);

		//has no further affect in the math...its just for graph compairson
		realAccel[0] = tempAccel[0];
		realAccel[1] = tempAccel[1];
		realAccel[2] = tempAccel[2];
		//---------------------------------------------------------------


		if (acceldeltaTime > 0)
		{
			if (abs(lastValidAccel[0] - tempAccel[0]) <= accelErrorWindow)
			{
				accel[0] = lastValidAccel[0];
				recalibrationCounterX++;
			}
			else
			{
				accel[0] = tempAccel[0];
				lastValidAccel[0] = accel[0];
				recalibrationCounterX = 0;
			}

			if (abs(lastValidAccel[1] - tempAccel[1]) <= accelErrorWindow)
			{
				accel[1] = lastValidAccel[1];
				recalibrationCounterY++;
			}
			else
			{
				accel[1] = tempAccel[1];
				lastValidAccel[1] = accel[1];
				recalibrationCounterY = 0;
			}

			if (abs(lastValidAccel[2] - tempAccel[2]) <= accelErrorWindow)
			{
				accel[2] = lastValidAccel[2];

			}
			else
			{
				accel[2] = tempAccel[2];
				lastValidAccel[2] = accel[2];

			}
		}
#pragma endregion

		if (MARKERTRACKING_FLAG && (marker[0] != MarkerPosition[0] || marker[1] != MarkerPosition[1] || marker[2] != MarkerPosition[2]))
		{
			marker = MarkerPosition;
			USE_IMAGE = true;
			MARKERTRACKING_FLAG = false;
		}


		// TESTING BLOCK -----------------------------
		//if (USE_IMAGE)
		//{

		//	cv::Mat prediction = KF.predict();
		//	cv::Point predictPt(prediction.at<float>(0), prediction.at<float>(1));

		//	measurments(0) = MarkerPosition[0];
		//	measurments(1) = MarkerPosition[1];

		//	cv::Point measPt(measurments(0), measurments(1));
		//	posv.push_back(measPt);

		//	cv::Mat estimated = KF.correct(measurments);
		//	cv::Point statePt(estimated.at<float>(0), estimated.at<float>(1));

		//	kalmanv.push_back(statePt);

		//	/*client->kalmanPos[0] = estimated.at<float>(0);
		//	client->kalmanPos[1] = estimated.at<float>(1);
		//	client->kalmanPos[2] = MarkerPosition[2];*/
		//}
		//--------------------------------------------

#pragma region Theta Kalman Filter (Angle)
		if (gyrodeltaTime > 0 || USE_IMAGE)
		{
			float PreviousAngle = client->kalmanPos[2];// KFo.statePost.at<float>(0, 0) * 180 / CV_PI;
			int TimesAround = PreviousAngle / 360;
			float MarkerAngle = marker[2];
			float nearAngle = MarkerAngle + TimesAround * 360;
			float correctedMarkerAngle = 0;
			if (abs(PreviousAngle - (nearAngle + 360)) <= abs(PreviousAngle - nearAngle))
			{
				correctedMarkerAngle = nearAngle + 360;
			}
			else
			{
				if (abs(PreviousAngle - (nearAngle - 360)) <= abs(PreviousAngle - nearAngle))
					correctedMarkerAngle = nearAngle - 360;
				else
					correctedMarkerAngle = nearAngle;
			}


			Ao.at<float>(0, 0) = 1;
			Ao.at<float>(0, 1) = gyrodeltaTime;
			Ao.at<float>(1, 0) = 0;
			Ao.at<float>(1, 1) = 1;

			Zo.at<float>(0, 0) = correctedMarkerAngle * CV_PI / 180;
			Zo.at<float>(1, 0) = 0;//gyro[2]; // rads/s

			if (USE_IMAGE)
				Ho.at<float>(0, 0) = 1;
			else
				Ho.at<float>(0, 0) = 0;
			Ho.at<float>(0, 1) = 0;
			Ho.at<float>(1, 0) = 0;
			Ho.at<float>(1, 1) = 1;

			KFo.transitionMatrix = Ao;
			KFo.measurementMatrix = Ho;

			cv::setIdentity(KFo.processNoiseCov, cv::Scalar(gyroError*CV_PI / 180)); //Q
			cv::setIdentity(KFo.measurementNoiseCov, cv::Scalar(0)); //R

			cv::Mat predictiono = KFo.predict();
			cv::Mat estimatedo = KFo.correct(Zo);

			//cout << "Xo^ =" << KFo.statePre << endl;
			//cout << "Xo^+1 =" << KFo.statePost << endl;
			//cout << "K = " << KFo.gain << endl << endl;;

			float radToDegrees = estimatedo.at<float>(0, 0) * 180 / CV_PI;
			client->kalmanPos[2] = fmod(radToDegrees, 360);
			lastGyroTS = gyroTS;
		}
#pragma endregion

		if (acceldeltaTime > 0 || USE_IMAGE)
		{
#pragma region X Kalman Filter

			//Ax =	| 1		delta(Time)  delta(time)^2   |
			//		| 0			1				1		 |
			//		| 0			0				1		 |
			Ax.at<float>(0, 0) = 1;
			Ax.at<float>(0, 1) = acceldeltaTime;
			Ax.at<float>(0, 2) = 0.5*(acceldeltaTime*acceldeltaTime);
			Ax.at<float>(1, 0) = 0;
			Ax.at<float>(1, 1) = 1;
			Ax.at<float>(1, 2) = acceldeltaTime;
			Ax.at<float>(2, 0) = 0;
			Ax.at<float>(2, 1) = 0;
			Ax.at<float>(2, 2) = 1;

			float tempaccelx = accel[0] * cos(client->kalmanPos[2] * CV_PI / 180) - accel[1] * sin(client->kalmanPos[2] * CV_PI / 180);
			if (recalibrationCounterX > xTries) //if the device is stopped (no acceleration) then the speed is reset to 0
			{
				Ax.at<float>(1, 1) = 0;
				Ax.at<float>(1, 2) = 0;
				Ax.at<float>(2, 2) = 0;
				if (tempaccelx < 0.05)
					tempaccelx = 0;
			}

			else
			{
				Ax.at<float>(1, 1) = 1;
				Ax.at<float>(1, 2) = acceldeltaTime;
				Ax.at<float>(2, 2) = 1;
			}

			Zx.at<float>(0, 0) = marker[0];
			prevXAccel = prevXAccel*(1 - lowPassFilterVar) + (tempaccelx)*lowPassFilterVar;
			Zx.at<float>(1, 0) = prevXAccel * 100;
			Zx.at<float>(2, 0) = 0;

			//Ux.at<float>(0, 0) = -accel[1] * 100;

			//Ux.at<float>(0, 0) = (-accel[1] * cos(estimatedo.at<float>(0, 0)) + accel[2] * sin(estimatedo.at<float>(0, 0))) * 100;
			//float tempaccelx = accel[1] * cos(client->kalmanPos[2] * CV_PI / 180) - accel[2] * sin(client->kalmanPos[2] * CV_PI / 180);

			Hx.zeros(3, 3, CV_32F);

			if (USE_IMAGE)
				Hx.at<float>(0, 0) = 1;
			else
				Hx.at<float>(0, 0) = 0;

			Hx.at<float>(0, 1) = 0;
			Hx.at<float>(0, 2) = 0;

			Hx.at<float>(1, 0) = 0;
			Hx.at<float>(1, 1) = 0;
			Hx.at<float>(1, 2) = 1; //accel;

			Hx.at<float>(2, 0) = 0; // wifi
			Hx.at<float>(2, 1) = 0;
			Hx.at<float>(2, 2) = 0;

			KFx.transitionMatrix = Ax;
			KFx.measurementMatrix = Hx;

			measurmentNoiseMatx.at<float>(0, 0) = 0.01;
			measurmentNoiseMatx.at<float>(0, 1) = 0;
			measurmentNoiseMatx.at<float>(0, 2) = 0;

			measurmentNoiseMatx.at<float>(1, 0) = 0;
			measurmentNoiseMatx.at<float>(1, 1) = xProcessError;
			measurmentNoiseMatx.at<float>(1, 2) = 0;

			measurmentNoiseMatx.at<float>(2, 0) = 0;
			measurmentNoiseMatx.at<float>(2, 1) = 0;
			measurmentNoiseMatx.at<float>(2, 2) = 9999;

			cv::setIdentity(KFx.processNoiseCov, cv::Scalar(accelError)); //Q
			//cv::setIdentity(KFy.measurementNoiseCov, cv::Scalar(yProcessError));
			KFx.measurementNoiseCov = measurmentNoiseMatx;

			cv::Mat predictionx = KFx.predict();

			cv::Mat estimatedx = KFx.correct(Zx);

#pragma endregion

#pragma region Y Kalman Filter
			//Ay =	| 1		delta(Time)  delta(time)^2   |
			//		| 0			1				1		 |
			//		| 0			0				1		 |
			Ay.at<float>(0, 0) = 1;
			Ay.at<float>(0, 1) = acceldeltaTime;
			Ay.at<float>(0, 2) = 0.5*(acceldeltaTime*acceldeltaTime);
			Ay.at<float>(1, 0) = 0;
			Ay.at<float>(1, 1) = 1;
			Ay.at<float>(1, 2) = acceldeltaTime;
			Ay.at<float>(2, 0) = 0;
			Ay.at<float>(2, 1) = 0;
			Ay.at<float>(2, 2) = 1;

			float tempaccely = accel[0] * sin(client->kalmanPos[2] * CV_PI / 180) + accel[1] * cos(client->kalmanPos[2] * CV_PI / 180);
			if (recalibrationCounterY > yTries)
			{
				Ay.at<float>(1, 1) = 0;
				Ay.at<float>(1, 2) = 0;
				Ay.at<float>(2, 2) = 0;
				if (tempaccely < 0.05)
					tempaccely = 0;
			}
			else
			{
				Ay.at<float>(1, 1) = 1;
				Ay.at<float>(1, 2) = acceldeltaTime;
				Ay.at<float>(2, 2) = 1;
			}

			Zy.at<float>(0, 0) = marker[1];
			prevYAccel = prevYAccel*(1 - lowPassFilterVar) + (tempaccely)*lowPassFilterVar;
			Zy.at<float>(1, 0) = prevYAccel * 100;
			Zy.at<float>(2, 0) = 0;// ver como é com o beacon

			//Uy.at<float>(0, 0) = accel[2] * 100;

			//Uy.at<float>(0, 0) = (accel[2] * cos(estimatedo.at<float>(0, 0)) - (-accel[1])*sin(estimatedo.at<float>(0, 0))) * 100;

			Hy.zeros(3, 3, CV_32F);

			if (USE_IMAGE)
				Hy.at<float>(0, 0) = 1;
			else
				Hy.at<float>(0, 0) = 0;

			Hy.at<float>(0, 1) = 0;
			Hy.at<float>(0, 2) = 0;

			Hy.at<float>(1, 0) = 0;
			Hy.at<float>(1, 1) = 0;
			Hy.at<float>(1, 2) = 1; //accel;

			Hy.at<float>(2, 0) = 0; // wifi
			Hy.at<float>(2, 1) = 0;
			Hy.at<float>(2, 2) = 0;

			KFy.transitionMatrix = Ay;
			KFy.measurementMatrix = Hy;

			measurmentNoiseMaty.zeros(3, 3, CV_32F);

			measurmentNoiseMaty.at<float>(0, 0) = 1;
			measurmentNoiseMaty.at<float>(0, 1) = 0;
			measurmentNoiseMaty.at<float>(0, 2) = 0;

			measurmentNoiseMaty.at<float>(1, 0) = 0;
			measurmentNoiseMaty.at<float>(1, 1) = yProcessError;
			measurmentNoiseMaty.at<float>(1, 2) = 0;

			measurmentNoiseMaty.at<float>(2, 0) = 0;
			measurmentNoiseMaty.at<float>(2, 1) = 0;
			measurmentNoiseMaty.at<float>(2, 2) = 9999;

			cv::setIdentity(KFy.processNoiseCov, cv::Scalar(accelError)); //Q
			cv::setIdentity(KFy.measurementNoiseCov, cv::Scalar(yProcessError));
			//KFy.measurementNoiseCov = measurmentNoiseMaty;

			cv::Mat predictiony = KFy.predict();
			cv::Mat estimatedy = KFy.correct(Zy);
#pragma endregion

			//RETREIVE RESULTS--------------
			client->kalmanPos[0] = estimatedx.at<float>(0, 0);
			client->kalmanPos[1] = estimatedy.at<float>(0, 0);
			lastAccelTS = accelTS;

			if (LOG_FILE_DEBUG)
			{
				string logText("" + to_string(acceldeltaTime) + " "																//acceleration delta time acceleration timestamp
					+ to_string(Zx.at<float>(1, 0)) + " " + to_string(Zy.at<float>(1, 0)) + " " + to_string(realAccel[2]) + " " // accelerationX accelerationY accelerationZ 
					+ to_string(KFx.statePost.at<float>(1, 0)) + " " + to_string(KFy.statePost.at<float>(1, 0)) + " "			// VelocityX VelocityY
					+ to_string(KFx.statePost.at<float>(0, 0)) + " " + to_string(KFy.statePost.at<float>(0, 0)) + " "			// UpdatedPositionX UpdatedPositionY
					+ to_string(KFx.statePre.at<float>(0, 0)) + " " + to_string(KFy.statePre.at<float>(0, 0)) + " "				// PredictedPositionX PredictedPositionY
					+ to_string(realAccel[0]) + " " + to_string(realAccel[1]) + " " + to_string(realAccel[2]) + " "				// RealAccelX RealAccelY RealAccelZ
					+ to_string(gyrodeltaTime) + " "																			// Gyro delta time
					+ to_string(KFo.statePost.at<float>(1, 0)) + " "																// Angular Velocity
					+ to_string(KFo.statePost.at<float>(0, 0)) + " "																// Angle
					+ to_string(gyro[0]) + " " + to_string(gyro[1]) + " " + to_string(gyro[2]) + " "							// Real Angular Velocity
					+ to_string(rawIncl[0]) + " " + to_string(-rawIncl[1]) + " " + to_string(rawIncl[2]) + " "		// InclinationX InclinationY InclinationZ
					+ to_string(USE_IMAGE ? 1 : 0) + " "																		// MarkerTracked (true or false)
					+ to_string(USE_IMAGE ? marker[0] : 0) + " " + to_string(USE_IMAGE ? marker[1] : 0) + " " + to_string(USE_IMAGE ? marker[2] : 0) + " " // MarkerX MarkerY MarkerO
					+ to_string(accelTS) + " "
					+ to_string(gyroTS) + " "
					+ to_string(rawAccel[0]) + " " + to_string(rawAccel[1]) + " " + to_string(rawAccel[2]) + " "
					+ to_string(rawGyro[0]) + " " + to_string(rawGyro[1]) + " " + to_string(rawGyro[2]));
				//ts ax ay az vx vy x y x' y' realX realY realZ gyroTS vo o realox realoy realoz incx incy incz marker markerx markery markero accelTS gyroTS

				//string logText("" + to_string(acceldeltaTime) + " "
				//	+ to_string(KFx.statePre.at<float>(0, 0)) + " " + to_string(KFy.statePre.at<float>(0, 0)) + " " + to_string(USE_IMAGE ? 1 : 0)); //ts x' y'
				auto iter = logFiles.find("log_file.txt");
				write_text_to_log_file(logText, iter->first, (iter->second));

				//TESTING BLOCK --------------------------------
				//string logText2("" + to_string(acceldeltaTime) + " " + to_string(KF.statePre.at<float>(0, 0)) + " " + to_string(KF.statePre.at<float>(1, 0))); //ts x' y'
				//auto iter2 = logFiles.find("marker_log_file.txt");
				//write_text_to_log_file(logText2, iter2->first, (iter2->second));
				//----------------------------------------------
			}

			stringStream.str("");
			stringStream
				<< "-----------VARIABLES------------" << endl
				<< "Ax =\n" << KFx.transitionMatrix << endl
				//<< "Bx =\n" << KFx.controlMatrix << endl
				//<< "Ux =\n" << Ux << endl
				<< "Zx =\n" << Zx << endl
				<< "Qx =\n" << KFx.processNoiseCov << endl
				<< "Rx =\n" << KFx.measurementNoiseCov << endl
				<< "Hx =\n" << KFx.measurementMatrix << endl

				<< "-----------PREDICTION-----------" << endl
				<< "Xx^ =\n" << KFx.statePre << endl
				<< "Px^ =\n" << KFx.errorCovPre << endl

				<< "-------------UPDATE-------------" << endl
				<< "Xx^+1 =\n" << KFx.statePost << endl
				<< "K =\n " << KFx.gain << endl << endl
				<< "Px^+1 =\n" << KFx.errorCovPost << endl;

			LoggingConsole.cls();
			LoggingConsole.printf(stringStream.str().c_str());
		}


		sampleTime = GetTickCount() - sampleTime;
		if (sampleTime > 1)
		{
			samples++;

			runningAverage = runningAverage * ((samples - 1) / samples) + (sampleTime / samples);

			client->samplingAverage = runningAverage;

			if (sampleTime > client->samplingMAX)
				client->samplingMAX = sampleTime;

			if (sampleTime < client->samplingMIN && sampleTime > 0)
				client->samplingMIN = sampleTime;
		}

		next_iter_tick += SKIP_TICKS;
		sleep_time = next_iter_tick - GetTickCount();
		if (sleep_time >= 0)
		{
			//std::cout << "Sleep Time " << sleep_time  << std::endl;
			Sleep(sleep_time);
		}
		ips++;
		if (USE_IMAGE)
		{

			USE_IMAGE = false;
		}

	}
}

#pragma endregion