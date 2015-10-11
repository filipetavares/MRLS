// MRLS.cpp : main project file.
#pragma region INCLUDES
#include "stdafx.h"
#include <WinSock2.h>
#include <windows.h>
#include <iostream> //console input/ouput
#include "aruco.h" // aruco lib
#include <map> //hashmap
#include <thread>
#include "Client.h"
#include "TestingClient.h"
#pragma endregion

#include "vld.h" // memory leaks detector

#pragma comment(lib, "ws2_32.lib") 

#pragma region NAMESPACES
#pragma endregion

#pragma region FUNCTIONS DECLARATIONS
bool processArgs(int argc, char** argv);
void validParameters();
void defaultParameters();
void consoleLoop();
void ListenTCPConnections(int port);
void closeConnection(SOCKET* socket);
void ListenToOutputTCPConnections(int port);
void sendOutputData();
void ExitFunction();
BOOL WINAPI ConsoleHandler(DWORD CEvent);
#pragma endregion

#pragma region GLOBAL VARIABLES
std::map<int, void*> ClientMap; // <id, Client>
std::map<int, SOCKET> outputClientMap;

thread ListenTCPThread, SendDataThread, ListenOutputTCPThread;

SOCKET ListenSocketTCP; //socket used to listen to incomming connections from clients... uses the inTCP port;
SOCKET ListenSocketTCPImage; // socket to receive image...
SOCKET ListenSocketTCPOutput; // socket to listen to output clients

int ClientIDCounter = 0;

int outputConnections = 0;

TestingClient *tc;

#pragma endregion

#pragma region INITIALIZATION GLOBAL VARIABLES
float MarkerSize = 0;	// size of the markers in the environment
float InitialPosition[] = { 0, 0 }; // the initial position for all the clients
std::string TheIntrinsicsFile = "default_camera_params.yml"; // default intrisics camera file for all clients
int CameraIDX = -1; // default camera IDX for live feed
int inTCPPort = 4671;	// default Input TCP port
int outTCPPort = 6881;	// default OutPut TCP Port
int inIMAGEPort = 10000;
#pragma endregion


#pragma region CONSOLE_EVENT_TRAP
BOOL WINAPI ConsoleHandler(DWORD CEvent)
{
	switch (CEvent)
	{
	case CTRL_CLOSE_EVENT:
		ExitFunction();
		break;
	default:
		break;
	}
	return TRUE;
}
#pragma endregion


#pragma region Main()
int main(int argc, char **argv)
{

	//setting a console trap to catch the close event from closing the console window throught the 'X'
	if (SetConsoleCtrlHandler(
		(PHANDLER_ROUTINE)ConsoleHandler, TRUE) == FALSE)
	{
		// unable to install handler... 
		// display message to the user
		printf("Unable to install handler!\n");
		return -1;
	}

	if (processArgs(argc, argv) == false) // in case the arguments are invalid!
	{
		validParameters();
		return 0;
	}

	ListenTCPThread = thread(ListenTCPConnections, inTCPPort); //creates a thread that listen to connections from clients
	ListenOutputTCPThread = thread(ListenToOutputTCPConnections, outTCPPort); // creates a thread that listen to connections to send information

	consoleLoop();

	closeConnection(&ListenSocketTCP); //close the client listenning socket
	closeConnection(&ListenSocketTCPOutput);

	
	ListenTCPThread.join(); //waits for the thread to sync
	if (SendDataThread.joinable())
		SendDataThread.join();
	if (ListenOutputTCPThread.joinable())
		ListenOutputTCPThread.join();

	ExitFunction();

	WSACleanup(); //cleans all the memory related to the sockets and connections
	return 0;
}

void ExitFunction()
{
	for (auto iter = ClientMap.begin(); iter != ClientMap.end(); iter++) //terminates all the clients
	{
		Client* c = (Client*)(iter->second);
		delete c;
		//ClientMap.erase(iter);
	}
	ClientMap.clear(); //make sure that the hash map is clean
	if (tc)
		delete tc;
}

#pragma endregion


#pragma region Arguments Processing
bool processArgs(int argc, char** argv)
{
	
	if (argc > 17) // if there are more arguments than the valid ones
	{
		std::cerr << "Invalid Number of Arguments!" << std::endl;
		return false;
	}
	
	if (argc == 1) // in case no argument is passed
	{
		defaultParameters();
		return true;
	}

	for (int i = 1; i < argc;) // Check which arguments where passed regardless the order
	{
		//std::cout << "i = " << i << endl;
		//std::cout << argv[i] << endl;
		string arg = argv[i];
		if (arg == "-ms")
		{
			try
			{
				MarkerSize = atof(argv[i + 1]);
				i = i + 2;
			}
			catch (std::exception &ex)
			{
				std::cerr << "The Marker Size was Invalid!" << std::endl;
				return false;
			}
		}
		else if (arg == "-pos")
		{
			try
			{
				InitialPosition[0] = atoi(argv[i + 1]);
				InitialPosition[1] = atoi(argv[i + 2]);
				i = i + 3;
			}
			catch (std::exception &ex)
			{
				std::cerr << "The Initial Position was Invalid!" << std::endl;
				return false;
			}
		}
		else if (arg == "-intrinsics")
		{
			try
			{
				aruco::CameraParameters cp;
				cp.readFromXMLFile(argv[i + 1]);
				TheIntrinsicsFile = atoi(argv[i + 1]);
				i = i + 2;
			}
			catch (std::exception &ex)
			{
				std::cerr << ex.what() << std::endl;
				return false;
			}
		}
		else if (arg == "-live")
		{
			try
			{
				//TODO : FAZER AS COISAS PARA CRIAR UM CLIENTE COM CAMERA (deixar para ultimo)
				i = i + 2;
			}
			catch (std::exception &ex)
			{
				std::cerr << ex.what() << std::endl;
				return false;
			}
		}
		else if (arg == "-inTCP")
		{
			try
			{
				inTCPPort = atoi(argv[i]);
				i = i + 2;
			}
			catch (std::exception &ex)
			{
				std::cerr << "The Input TCP Port is Invalid!" << std::endl;
				return false;
			}
		}
		else if (arg == "-outTCP")
		{
			try
			{
				outTCPPort = atoi(argv[i]);
				i = i + 2;
			}
			catch (std::exception &ex)
			{
				std::cerr << "The Output TCP Port is Invalid!" << std::endl;
				return false;
			}
		}	
	}
	return true;
		
}

void validParameters()
{
	std::cerr << "The valid parameters are:" << std::endl;
	std::cerr << "-ms markerSize (in cm)" << std::endl;
	std::cerr << "-pos x y (in cm)" << std::endl;
	std::cerr << "-intrinsics [intrinsics_file.yml]" << std::endl;
	std::cerr << "-live idx (camera id)" << std::endl;
	std::cerr << "-inTCP port" << std::endl;
	std::cerr << "-outTCP port" << std::endl;
	std::cerr << "-inUDP port" << std::endl;
	std::cerr << "-outUDP port" << std::endl;
}

void defaultParameters()
{
	//TODO: set default parameters
	std::cerr << "Default Parameters Set:" << std::endl;
	std::cerr << "Marker Size = " << MarkerSize  << "cm" << std::endl;
	std::cerr << "Initial Position = (" << InitialPosition[0] << ", " << InitialPosition[1] << ")" << std::endl;
	std::cerr << "Default Intrinsics File = " << TheIntrinsicsFile << std::endl;
	std::cerr << "Live Camera (COM X) = NONE" << std::endl; // alterar este para ter uma logica de reconhecimento
	std::cerr << "In TCP Port = " << inTCPPort << std::endl;
	std::cerr << "Out TCP Port = " << outTCPPort << std::endl;
	std::cerr << "In Image Port = " << inIMAGEPort << std::endl;
}
#pragma endregion


#pragma region CONSOLE COMMANDS
void consoleLoop()
{
	//TODO Console Interaction
	defaultParameters();
	std::string consoleCommand = "";
	while (consoleCommand != "exit" && consoleCommand != "quit")
	{
		cout << ">";
		getline(cin, consoleCommand);

		if (consoleCommand == "cls") //clear screen command
		{
			system("cls");
		}
			
		else if (consoleCommand == "connections") //checks how many clients are connected
		{
			cout << ClientMap.size() << " clients connected" << endl;
			cout << outputConnections << " Output Clients Connected" << endl;
		}
		else if (consoleCommand == "exit" || consoleCommand == "quit") //quit command
		{
			//ExitFunction();
			break;
		}
		else if (consoleCommand == "list") //lists all the clients
		{
			for (auto iter = ClientMap.begin(); iter != ClientMap.end(); iter++)
			{
				Client* c = (Client*)(iter->second);
				cout << "[Client] ID:" << c->getID() << endl;
			}
		}
		else if (consoleCommand.find("client ") != string::npos) // displays a specific client by ID
		{
			string buff;
			stringstream stream(consoleCommand);
			vector<string> tokens;
			while (stream >> buff)
				tokens.push_back(buff);


			auto iter = ClientMap.find(atoi(tokens[1].c_str()));
			if (iter == ClientMap.end())
			{
				cout << "CLIENT ID NOT FOUND!" << endl;
				continue;
			}
			Client* c = (Client*)(iter->second);

			cout << "[Client]" << endl;
			cout << "ID:\t\t" << c->getID() << endl;
			cout << "MarkerTracking Iterations:\t" << c->tracker->MTPS << "/s" << endl;
			cout << "Average Sampling Time = " << c->samplingAverage << "  MAX = " << c->samplingMAX << "  MIN = " << c->samplingMIN << endl;
			cout << "Kalman Iterations:\t" << c->tracker->KalmanPS << "/s" << endl;
			cout << "Position\t(" << c->getPosition()[0] << ", " << c->getPosition()[1] << ", " << c->getPosition()[2] << ")" << endl;
			cout << "Kalman Pos\t(" << c->kalmanPos[0] << ", " << c->kalmanPos[1] << ", " << c->kalmanPos[2] << ")" << endl;
			cout << "Orientation\t(" << (int)(c->orientation[1] * 180 / CV_PI) << ", " << (int)(c->orientation[2] * 180 / CV_PI) << ", " << (int)(c->orientation[0] * 180 / CV_PI) << ")" << endl;
			//cout << "OrientationRAW\t(" << c->orientation[0] << ", " << c->orientation[1] << ", " << c->orientation[2] << ")" << endl;
			cout << "Gyro\t\t(" << c->gyro[0] << ", " << c->gyro[1] << ", " << c->gyro[2] << ")" << endl;
			cout << "Accel\t\t(" << c->accel[0] << ", " << c->accel[1] << ", " << c->accel[2] << ")" << endl;
			cout << "Wifi RSSI = " << c->wifiRssi << " freq = " << c->wifiFreq << " -> " << pow(10.0, ((27.55 - (20*log10(c->wifiFreq)) + abs(c->wifiRssi))/20)) << endl;
			cout << "Marker\t\tID: " << c->markerID << " Distance: " << c->distanceFromMarker << "cm" << endl;
		}
		else if (consoleCommand.find("delete ") != string::npos) //deletes a specific client by ID
		{
			string buff;
			stringstream stream(consoleCommand);
			vector<string> tokens;
			while (stream >> buff)
				tokens.push_back(buff);

			auto iter = ClientMap.find(atoi(tokens[1].c_str()));
			if (iter == ClientMap.end())
			{
				cout << "CLIENT ID NOT FOUND!" << endl;
				continue;
			}
			Client* c = (Client*)(iter->second);
			delete c;
			ClientMap.erase(iter);
		}
		else if (consoleCommand.find("display ") != string::npos) //displays the image of a specific client by ID
		{
			string buff;
			stringstream stream(consoleCommand);
			vector<string> tokens;
			while (stream >> buff)
				tokens.push_back(buff);

			auto iter = ClientMap.find(atoi(tokens[1].c_str()));
			if (iter == ClientMap.end())
			{
				cout << "CLIENT ID NOT FOUND!" << endl;
				continue;
			}
			Client* c = (Client*)(iter->second);
			c->DisplayImage();
		}
		else if (consoleCommand.find("track ") != string::npos) //displays the image of a specific client by ID
		{
			string buff;
			stringstream stream(consoleCommand);
			vector<string> tokens;
			while (stream >> buff)
				tokens.push_back(buff);

			auto iter = ClientMap.find(atoi(tokens[1].c_str()));
			if (iter == ClientMap.end())
			{
				cout << "CLIENT ID NOT FOUND!" << endl;
				continue;
			}
			Client* c = (Client*)(iter->second);
			c->startTracking();
		}
		else if (consoleCommand.find("test") != string::npos)
		{
			if (!tc)
			{
				tc = new TestingClient();
			}

			string buff;
			stringstream stream(consoleCommand);
			vector<string> tokens;
			while (stream >> buff)
				tokens.push_back(buff);

			for (int i = 1; i < tokens.size();)
			{
				if (tokens[i] == "-rf")
				{
					if (i + 1 >= tokens.size() || tokens[i + 1].substr(0, 1) == "-")
					{
						cout << "-rf -> No Log File path provided. Default Log File path is \"log_file.txt\" "  << endl;
						tc->readFile("log_file.txt");
						i++;
					}
					else
					{
						tc->readFile(tokens[i + 1]);
						i = i + 2;
					}
					
				}
				else if (tokens[i] == "-run")
				{
					if (i+1 >= tokens.size() || tokens[i + 1].substr(0, 1) == "-")
					{
						cout << "-run -> No Log File path provided. Default Log File path is \"testLogFile.txt\" " << endl;
						tc->ExtendedKalmanFilter();
						i++;
					}
					else
					{
						tc->ExtendedKalmanFilter(tokens[i + 1]);
						i = i + 2;
					}
				}
				else if (tokens[i] == "-lp")
				{
					if (tokens[i + 1].substr(0, 1) != "-")
					{
						float lpv = 0;
						try
						{
							lpv = stof(tokens[i + 1]);
						}
						catch(std::exception &e)
						{
							cout << "-lp -> Invalid Argument!" << endl;
							break;
						}

						if (lpv >= 0 && lpv <= 1.0)
						{
							tc->lowPassFilterVar = lpv;
							cout << "-lp -> Low Pass Variable set to " << lpv << endl;
							i = i + 2;
						}
						else
						{
							cout << "-lp -> Invalid Argument! The number must be between 0.0 - 1.0." << endl;
							break;
						}
					}
					else
					{
						cout << "-lp -> Invalid Argument!" << endl;
					}
				}
				else if (tokens[i] == "-aew")
				{
					if (tokens[i + 1].substr(0, 1) != "-")
					{
						float lpv = 0;
						try
						{
							lpv = stof(tokens[i + 1]);
						}
						catch (std::exception &e)
						{
							cout << "-aew -> Invalid Argument!" << endl;
							break;
						}

						if (lpv >= 0 && lpv <= 1.0)
						{
							tc->accelErrorWindow = lpv;
							cout << "-aew -> Acceleration Error Window set to " << lpv << endl;
							i = i + 2;
						}
						else
						{
							cout << "-aew -> Invalid Argument! The number must be between 0.0 - 1.0." << endl;
							break;
						}
					}
					else
					{
						cout << "-aew -> Invalid Argument!" << endl;
					}
				}
				else if (tokens[i] == "-gew")
				{
					if (tokens[i + 1].substr(0, 1) != "-")
					{
						float lpv = 0;
						try
						{
							lpv = stof(tokens[i + 1]);
						}
						catch (std::exception &e)
						{
							cout << "-gew -> Invalid Argument!" << endl;
							break;
						}

						if (lpv >= 0 && lpv <= 1.0)
						{
							tc->gyroErrorWindow = lpv;
							cout << "-gew -> Gyro Error Window set to " << lpv << endl;
							i = i + 2;
						}
						else
						{
							cout << "-gew -> Invalid Argument! The number must be between 0.0 - 1.0." << endl;
							break;
						}
					}
					else
					{
						cout << "-gew -> Invalid Argument!" << endl;
					}
				}
				else
				{
					cout << "Invalid Command!" << endl;
					break;
				}
			}
			
		}
	}
}


#pragma endregion

#pragma region NETWORK SERVER FUNCTIONS
void ListenTCPConnections(int port)
{
	//Initialize Winsock
	WSADATA wsaData;
	int iResult = 0;

	ListenSocketTCP = INVALID_SOCKET;
	SOCKET ImageSocket = INVALID_SOCKET;
	sockaddr_in TCPservice, UDPService;

	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != NO_ERROR)
	{
		wprintf(L"WSAStartup() failed with error: %d\n", iResult);
		return;
	}

	/*-------- Creates both TCP sockets for the client to connect--------------*/
	/*-------------------------------------------------------------------------*/
	/*---------------------Sensor Data Socket Setup----------------------------*/
	/*-------------------------------------------------------------------------*/
	//Create a SOCKET for Listenting for incoming TCP connections requests.
	ListenSocketTCP = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (ListenSocketTCP == INVALID_SOCKET)
	{
		wprintf(L"socket function failed with error: %ld\n", WSAGetLastError());
		WSACleanup();
		return;
	}

	// The sockaddr_in structure specifies the address family,
	// IP address, and port for the socket that is being bound.
	TCPservice.sin_family = AF_INET;
	TCPservice.sin_addr.s_addr = INADDR_ANY;
	TCPservice.sin_port = htons(port);

	iResult = ::bind(ListenSocketTCP, (SOCKADDR *)& TCPservice, sizeof (TCPservice));
	if (iResult == SOCKET_ERROR) {
		wprintf(L"bind function failed with error %d\n", WSAGetLastError());
		iResult = closesocket(ListenSocketTCP);
		if (iResult == SOCKET_ERROR)
			wprintf(L"closesocket function failed with error %d\n", WSAGetLastError());
		WSACleanup();
		return;
	}

	// Listen for incoming connection requests 
	// on the created socket
	if (listen(ListenSocketTCP, 5) == SOCKET_ERROR)
		wprintf(L"listen function failed with error: %d\n", WSAGetLastError());

	/*-------------------------------------------------------------------------*/
	/*---------------------Image Data Socket Setup----------------------------*/
	/*-------------------------------------------------------------------------*/

	//Create a SOCKET for Listenting for incoming TCP connections requests.
	ImageSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (ListenSocketTCP == INVALID_SOCKET)
	{
		wprintf(L"socket function failed with error: %ld\n", WSAGetLastError());
		WSACleanup();
		return;
	}

	// The sockaddr_in structure specifies the address family,
	// IP address, and port for the socket that is being bound.
	TCPservice.sin_family = AF_INET;
	TCPservice.sin_addr.s_addr = INADDR_ANY;
	TCPservice.sin_port = htons(inIMAGEPort);

	iResult = ::bind(ImageSocket, (SOCKADDR *)& TCPservice, sizeof (TCPservice));
	if (iResult == SOCKET_ERROR) {
		wprintf(L"bind function failed with error %d\n", WSAGetLastError());
		iResult = closesocket(ImageSocket);
		if (iResult == SOCKET_ERROR)
			wprintf(L"closesocket function failed with error %d\n", WSAGetLastError());
		WSACleanup();
		return;
	}

	// Listen for incoming connection requests 
	// on the created socket
	if (listen(ImageSocket, 5) == SOCKET_ERROR)
		wprintf(L"listen function failed with error: %d\n", WSAGetLastError());


	SOCKET dataSock = INVALID_SOCKET;
	SOCKET imageSock = INVALID_SOCKET;

	while (true)
	{

		dataSock = accept(ListenSocketTCP, NULL, NULL);	//accpet a client
		if (ListenSocketTCP == INVALID_SOCKET) //if the socket gets closed, we stop listening
			break;

		imageSock = accept(ImageSocket, NULL, NULL);	//accpet a client
		if (ListenSocketTCP == INVALID_SOCKET) //if the socket gets closed, we stop listening
			break;


		Client* c = new Client(dataSock, imageSock, ClientIDCounter++, 0, 0, 0, TheIntrinsicsFile, MarkerSize); //initialize the client
		ClientMap.insert(std::pair<int, void*>(c->getID(), (void*)c)); //put the client in the hash map
		c->start(); //start tracking the client
		cout << "Client ID:" << c->getID() << " Connected" << endl;
		//delete c;
		//TODO: Criar um novo client com a porta TCP e UDP
	}
	return;
}

void closeConnection(SOCKET* socket)
{
	if (*socket != INVALID_SOCKET)
	{
		closesocket(*socket); //close the socket
		*socket = INVALID_SOCKET; //flag the socket as closed
	}
}

void ListenToOutputTCPConnections(int port)
{
	//Initialize Winsock
	WSADATA wsaData;
	int iResult = 0;

	ListenSocketTCPOutput = INVALID_SOCKET;
	sockaddr_in TCPservice, UDPService;

	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != NO_ERROR)
	{
		wprintf(L"WSAStartup() failed with error: %d\n", iResult);
		return;
	}

	/*-------------------------------------------------------------------------*/
	/*-------------------------------------------------------------------------*/
	/*---------------------Output Data Socket Setup----------------------------*/
	/*-------------------------------------------------------------------------*/
	//Create a SOCKET to Listen for incoming TCP connections requests.
	ListenSocketTCPOutput = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (ListenSocketTCPOutput == INVALID_SOCKET)
	{
		wprintf(L"socket function failed with error: %ld\n", WSAGetLastError());
		WSACleanup();
		return;
	}

	// The sockaddr_in structure specifies the address family,
	// IP address, and port for the socket that is being bound.
	TCPservice.sin_family = AF_INET;
	TCPservice.sin_addr.s_addr = INADDR_ANY;
	TCPservice.sin_port = htons(port);

	iResult = ::bind(ListenSocketTCPOutput, (SOCKADDR *)& TCPservice, sizeof (TCPservice));
	if (iResult == SOCKET_ERROR) {
		wprintf(L"bind function failed with error %d\n", WSAGetLastError());
		iResult = closesocket(ListenSocketTCPOutput);
		if (iResult == SOCKET_ERROR)
			wprintf(L"closesocket function failed with error %d\n", WSAGetLastError());
		WSACleanup();
		return;
	}

	// Listen for incoming connection requests 
	// on the created socket
	if (listen(ListenSocketTCPOutput, 5) == SOCKET_ERROR)
		wprintf(L"listen function failed with error: %d\n", WSAGetLastError());

	SOCKET outputSock = INVALID_SOCKET;
	int socketID = 0;
	bool threadStarted = false;

	while (true)
	{

		outputSock = accept(ListenSocketTCPOutput, NULL, NULL);	//accpet a client
		if (ListenSocketTCPOutput == INVALID_SOCKET) //if the socket gets closed, we stop listening
			break;
		
		outputClientMap.insert(std::pair<int, SOCKET>(socketID, outputSock));
		socketID++;
		outputConnections++;

		if (!threadStarted)
		{
			SendDataThread = thread(sendOutputData);
			threadStarted = true;
		}
		//saves the socket to send the data to the client;
	}

	for (auto iter = outputClientMap.begin(); iter != outputClientMap.end(); iter++) // closes all outbound connections
	{
		SOCKET s = iter->second;
		closeConnection(&s);
	}

	outputClientMap.clear(); //clears the map for outbound connections
	return;
}

void sendOutputData()
{
	int iResult = 0;
	/*Json::Value sendMsg;
	Json::Value data;
	
	int arraySize = 0;*/
	const int FRAMES_PER_SECOND = 200;
	const int SKIP_TICKS = 1000 / FRAMES_PER_SECOND;

	DWORD next_game_tick = GetTickCount();
	int sleep_time = 0;

	std::string data;
	while (ListenSocketTCPOutput != INVALID_SOCKET)
	{
		next_game_tick += SKIP_TICKS;
		sleep_time = next_game_tick - GetTickCount();
		if (sleep_time >= 0)
		{
			//std::cout << "Sleep Time " << sleep_time  << std::endl;
			Sleep(sleep_time);
		}

		data = "";
		/*sendMsg.clear();
		data.clear();
		arraySize = 0;*/
		if (outputConnections < 1)
			continue;

		if (ClientMap.size() < 1)
			continue;

		for (auto iter = ClientMap.begin(); iter != ClientMap.end(); iter++)
		{
			try
			{

				Client* c = (Client*)(iter->second);
				//Json::Value	jsClient;
				//jsClient["ID"] = c->getID();
				//jsClient["Position"]["x"] = c->kalmanPos[0];
				//jsClient["Position"]["y"] = c->kalmanPos[1];
				//jsClient["Position"]["theta"] = c->kalmanPos[2];
				////data.append(jsClient);
				//arraySize++;
				data += "" + std::to_string(c->getID()) + ":"
					+ std::to_string(c->kalmanPos[0]) + ":"
					+ std::to_string(c->kalmanPos[1]) + ":"
					+ std::to_string(c->kalmanPos[2]) + "|";
			}
			catch (std::exception &e)
			{
				cout << e.what() << endl;
			}
		}

		data += "\n";
	/*	sendMsg["size"] = arraySize;
		sendMsg["data"] = data;*/
		//cout << sendMsg << endl;

		for (auto iter = outputClientMap.begin(); iter != outputClientMap.end(); iter++)
		{
			try
			{
				SOCKET s = iter->second;
				//cout << sendMsg.toStyledString().c_str() << endl;
				iResult = send(s, data.c_str(), data.size(), 0);
				if (iResult == SOCKET_ERROR && ListenSocketTCPOutput != INVALID_SOCKET) // if client is disconnected
				{
					//erase client
					closeConnection(&s);
					outputClientMap.erase(iter);
					outputConnections--;
				}
			}
			catch (std::exception &e)
			{
				cout << e.what() << endl;
			}

			if (outputClientMap.size() <= 0)
			{
				break;
			}
			
		}
		

	}

}
#pragma endregion