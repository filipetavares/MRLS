#include <Windows.h>
#include "stdafx.h"
#include "TestingClient.h"
#include <opencv\cv.h>
#include <thread>



TestingClient::TestingClient()
{
	Position[0] = 0;
	Position[1] = 0;
	Position[2] = 0;
	kalmanPos[0] = 0;
	kalmanPos[1] = 0;
	kalmanPos[2] = 0;
	accel[0] = 0;
	accel[1] = 0;
	accel[2] = 0;
	gyro[0] = 0;
	gyro[1] = 0;
	gyro[2] = 0;
	timestamp = 0;
}

TestingClient::~TestingClient()
{

}

void TestingClient::reRun(string logFileName = "testLogFile.txt")
{


	if (logFileName != "")
	{
		//TODO: escrever no logFile
	}
}

void TestingClient::readFile(std::string path)
{
	std::ifstream logFile(path);
	int lines = 0;
	int words = 0;

	if (!logFile)
	{
		std::cout << "No File Found! Please check the FILEPATH again." << std::endl;
		return;
	}

	cout << " Clearing Previous used Memory " << endl;
	accelDeltaTime.clear();
	acceleration.clear();
	velocity.clear();
	position.clear();
	predictedPosition.clear();
	realAcceleration.clear();
	gyroDeltaTime.clear();
	angularVelocity.clear();
	angle.clear();
	realAngularVelocity.clear();
	inclination.clear();
	markerTracked.clear();
	Marker.clear();
	accelTimestamp.clear();
	gyroTimestamp.clear();
	rawAcceleration.clear();
	rawGyro.clear();
	cout << " Memory cleared! " << endl;

	cout << " Reading " << path << "..." << endl;
	while (logFile)
	{
		string line, buffer;
		stringstream linestream;
		vector<string> tokens;
		getline(logFile, line);

		linestream << line;
		
		if (line == "")
			break;
		while (linestream >> buffer)
		{
			tokens.push_back(buffer);
			words++;
		}
		accelDeltaTime.push_back(stod(tokens[0]));
		acceleration.push_back(array<float, 3>{{ stof(tokens[1]), stof(tokens[2]), stof(tokens[3]) }});
		velocity.push_back(array<float, 2>{{ stof(tokens[4]), stof(tokens[5]) }});
		position.push_back(array<float, 2>{{ stof(tokens[6]), stof(tokens[7]) }});
		predictedPosition.push_back(array<float, 2>{{ stof(tokens[8]), stof(tokens[9]) }});
		realAcceleration.push_back(array<float, 3>{{ stof(tokens[10]), stof(tokens[11]), stof(tokens[12]) }});
		gyroDeltaTime.push_back(stod(tokens[13]));
		angularVelocity.push_back(stof(tokens[14]));
		angle.push_back(stof(tokens[15]));
		realAngularVelocity.push_back(array<float, 3>{{ stof(tokens[16]), stof(tokens[17]), stof(tokens[18]) }});
		inclination.push_back(array<float, 3>{{ stof(tokens[19]), stof(tokens[20]), stof(tokens[21]) }});
		markerTracked.push_back(stoi(tokens[22]));
		Marker.push_back(array<float, 3>{{ stof(tokens[23]), stof(tokens[24]), stof(tokens[25]) }});
		accelTimestamp.push_back(stod(tokens[26]));
		gyroTimestamp.push_back(stod(tokens[27]));
		rawAcceleration.push_back(array<float, 3>{{ stof(tokens[28]), stof(tokens[29]), stof(tokens[30]) }});
		rawGyro.push_back(array<float, 3>{{ stof(tokens[31]), stof(tokens[32]), stof(tokens[33]) }});

		tokens.clear();
		lines++;
	}
	cout << "Reading Complete!" << endl;
	cout << lines << " line processed." << endl;
	cout << words << " words processed." << endl;
	currentFile = path;
}

void TestingClient::write_text_to_log_file(const std::string &text, const std::string &filename, bool &fileStarted)
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

void TestingClient::ExtendedKalmanFilter(string logFileName)
{

	if (accelDeltaTime.size() < 1)
	{
		cout << "No File Loaded!" << endl;
	}

	cout << "Starting to run the Kalman Filter with the Following Parameters" << endl;
	cout << "File = " << currentFile << endl;
	cout << "Iterations per Second (not limited = 1000) = " << IPS << endl;
	cout << "Low Pass Filter Variable (OFF = 1) = " << lowPassFilterVar << endl;
	cout << "Acceleration Window Error (OFF = 0) = " << accelErrorWindow << endl;
	cout << "Acceleration Error = " << accelError << endl;
	cout << "Gyro Window Error (OFF = 0) = " << gyroErrorWindow << endl;
	cout << "Gyro Error = " << gyroError << endl;

	bool LOG_FILE_DEBUG = true;
	if (LOG_FILE_DEBUG)
		logFiles.insert(std::pair<std::string, boolean>(logFileName, false));

	//float lowPassFilterVar = 1;// 0.3; // Low Pass filter in case it is used (set 1 to turn off).

	cv::KalmanFilter KFo = cv::KalmanFilter(2, 2);
	cv::Mat Ao = cv::Mat_<float>(2, 2);
	cv::Mat Zo = cv::Mat_<float>(2, 1);
	cv::Mat Ho = cv::Mat_<float>(2, 2);
	cv::Mat estimatedo = cv::Mat_<float>(1, 1);
	estimatedo.at<float>(0, 0) = 0;
	KFo.statePost.at<float>(0, 0) = 0;
	float lastAngleInRads = 0;
	//float gyroError = 0.03;							//Gyro sensor measurment error

	cv::KalmanFilter KFx = cv::KalmanFilter(3, 3); // [x, vx] [1marker, wifi, (sensors?)] [ax(accelerometer)]
	cv::Mat Ax = cv::Mat_<float>(3, 3);
	cv::Mat Zx = cv::Mat_<float>(3, 1);
	cv::Mat Hx = cv::Mat_<float>(3, 3);
	float prevXAccel = 0;							//keeping track of the previous Xaccelaration
	//float accelError = 0.00022;//0.00022;						//Accelerator sensor Measurment Error



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

	const int ITERS_PER_SECOND = IPS;
	const int SKIP_TICKS = 1000 / ITERS_PER_SECOND;

	DWORD next_iter_tick = GetTickCount();
	int sleep_time = 0;
	bool kalman_is_running = true;
	DWORD startCount = GetTickCount();
	int ips = 0;

	float accel[3] = { 0, 0, 0 };
	float lastValidAccel[3] = { 0, 0, 0 };
	float realAccel[3] = { 0, 0, 0 };
	//float accelErrorWindow = 0.15;

	float gyro[3] = { 0, 0, 0 };
	float lastValidGyro[3] = { 0, 0, 0 };
	float orientation[3] = { 0, 0, 0 };
	//float gyroErrorWindow = 0.1;

	int recalibrationCounterX = 0;
	int xTries = 3;
	int recalibrationCounterY = 0;
	int yTries = 3;
	vector<float> marker = { 0, 0, 0 };

	int iterations = 0;

	while (iterations < accelDeltaTime.size())
	{

		int i = iterations;



		//if (debugInput && debugInput < 10)
		//{
		//	client->DebugInput(debugInputVar);
		//	debugInputVar++;
		//}

		//if (GetTickCount() - startCount > 1000) //Counts the iterations per second
		//{
		//	this->KalmanPS = ips;
		//	startCount = GetTickCount();
		//	ips = 0;
		//}

#pragma region Variables Update
		if (lastGyroTS == -1 && gyroTimestamp[i] > 1)
			lastGyroTS = gyroTimestamp[i] - 1;

		gyroTS = gyroTimestamp[i];
		gyro[0] = rawGyro[i][0];
		gyro[1] = rawGyro[i][1];
		gyro[2] = rawGyro[i][2];

		if (lastAccelTS == -1 && accelTimestamp[i] > 0)
			lastAccelTS = accelTimestamp[i] - 1;

		accelTS = accelTimestamp[i];
		accel[0] = rawAcceleration[i][0];
		accel[1] = rawAcceleration[i][1];
		accel[2] = rawAcceleration[i][2];

		//Inclination in relation to the floor
		orientation[0] = -inclination[i][0]; //x
		orientation[1] = inclination[i][1]; //y 
		orientation[2] = inclination[i][2]; //z 

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

		if (markerTracked[i] && (marker[0] != Marker[i][0] || marker[1] != Marker[i][1] || marker[2] != Marker[i][2]))
		{
			marker[0] = Marker[i][0];
			marker[1] = Marker[i][1];
			marker[2] = Marker[i][2];
			USE_IMAGE = true;
		}

#pragma region Theta Kalman Filter (Angle)
		if (gyrodeltaTime > 0 || USE_IMAGE)
		{
			float PreviousAngle = kalmanPos[2];// KFo.statePost.at<float>(0, 0) * 180 / CV_PI;
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
			cv::setIdentity(KFo.measurementNoiseCov, cv::Scalar(0.1)); //R

			cv::Mat predictiono = KFo.predict();
			cv::Mat estimatedo = KFo.correct(Zo);

			//cout << "Xo^ =" << KFo.statePre << endl;
			//cout << "Xo^+1 =" << KFo.statePost << endl;
			//cout << "K = " << KFo.gain << endl << endl;;

			float radToDegrees = estimatedo.at<float>(0, 0) * 180 / CV_PI;
			kalmanPos[2] = fmod(radToDegrees, 360);
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

			float tempaccelx = accel[0] * cos(kalmanPos[2] * CV_PI / 180) - accel[1] * sin(kalmanPos[2] * CV_PI / 180);
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

			cv::setIdentity(KFx.processNoiseCov, cv::Scalar(accelError)); //Q
			cv::setIdentity(KFx.measurementNoiseCov, cv::Scalar(0.0));

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

			float tempaccely = accel[0] * sin(kalmanPos[2] * CV_PI / 180) + accel[1] * cos(kalmanPos[2] * CV_PI / 180);
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

			cv::setIdentity(KFy.processNoiseCov, cv::Scalar(accelError)); //Q
			cv::setIdentity(KFy.measurementNoiseCov, cv::Scalar(0.1));

			cv::Mat predictiony = KFy.predict();
			cv::Mat estimatedy = KFy.correct(Zy);
#pragma endregion

			//RETREIVE RESULTS--------------
			kalmanPos[0] = estimatedx.at<float>(0, 0);
			kalmanPos[1] = estimatedy.at<float>(0, 0);
			lastAccelTS = accelTS;

			if (LOG_FILE_DEBUG)
			{
				string logText("" + to_string(acceldeltaTime) + " "																//acceleration delta time
					+ to_string(Zx.at<float>(1, 0)) + " " + to_string(Zy.at<float>(1, 0)) + " " + to_string(realAccel[2]) + " " // accelerationX accelerationY accelerationZ 
					+ to_string(KFx.statePost.at<float>(1, 0)) + " " + to_string(KFy.statePost.at<float>(1, 0)) + " "			// VelocityX VelocityY
					+ to_string(KFx.statePost.at<float>(0, 0)) + " " + to_string(KFy.statePost.at<float>(0, 0)) + " "			// UpdatedPositionX UpdatedPositionY
					+ to_string(KFx.statePre.at<float>(0, 0)) + " " + to_string(KFy.statePre.at<float>(0, 0)) + " "				// PredictedPositionX PredictedPositionY
					+ to_string(realAccel[0]) + " " + to_string(realAccel[1]) + " " + to_string(realAccel[2]) + " "				// RealAccelX RealAccelY RealAccelZ
					+ to_string(gyrodeltaTime) + " "																			// Gyro delta time
					+ to_string(KFo.statePost.at<float>(1, 0)) + " "																// Angular Velocity
					+ to_string(KFo.statePost.at<float>(0, 0)) + " "																// Angle
					+ to_string(gyro[0]) + " " + to_string(gyro[1]) + " " + to_string(gyro[2]) + " "							// Real Angular Velocity
					+ to_string(-orientation[0]) + " " + to_string(orientation[1]) + " " + to_string(orientation[2]) + " "		// InclinationX InclinationY InclinationZ
					+ to_string(USE_IMAGE ? 1 : 0) + " "																		// MarkerTracked (true or false)
					+ to_string(USE_IMAGE ? marker[0] : 0) + " " + to_string(USE_IMAGE ? marker[1] : 0) + " " + to_string(USE_IMAGE ? marker[2] : 0) + " " // MarkerX MarkerY MarkerO
					+ to_string(accelTimestamp[i]) + " "
					+ to_string(gyroTimestamp[i]) + " "
					+ to_string(rawAcceleration[i][0]) + " " + to_string(rawAcceleration[i][1]) + " " + to_string(rawAcceleration[i][2]) + " "
					+ to_string(rawGyro[i][0]) + " " + to_string(rawGyro[i][1]) + " " + to_string(rawGyro[i][2]));
				//ts ax ay az vx vy x y x' y' realX realY realZ gyroTS vo o realox realoy realoz incx incy incz marker markerx markery markero

				//string logText("" + to_string(acceldeltaTime) + " "
				//	+ to_string(KFx.statePre.at<float>(0, 0)) + " " + to_string(KFy.statePre.at<float>(0, 0)) + " " + to_string(USE_IMAGE ? 1 : 0)); //ts x' y'
				auto iter = logFiles.find(logFileName);
				write_text_to_log_file(logText, iter->first, (iter->second));

				//TESTING BLOCK --------------------------------
				//string logText2("" + to_string(acceldeltaTime) + " " + to_string(KF.statePre.at<float>(0, 0)) + " " + to_string(KF.statePre.at<float>(1, 0))); //ts x' y'
				//auto iter2 = logFiles.find("marker_log_file.txt");
				//write_text_to_log_file(logText2, iter2->first, (iter2->second));
				//----------------------------------------------
			}
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
		iterations++;
	}

	logFiles.clear();
	cout << "Done Running Kalman Filter!" << endl;
	cout << "Please check " << logFileName << endl;
}

