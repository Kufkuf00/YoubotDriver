#include "MTaskFreeDrive.hpp"
#include "MotionLayer.hpp"
#include "Logger.hpp"


#include <iostream>
#include <fstream>
#include <cmath>

using namespace youbot;

MTask::TaskType youbot::MTaskFreeDrive::GetType() const
{
	return TaskType();
}
/*youbot::MTaskFreeDrive::MTaskFreeDrive() {
	MyFile("filename.csv");
}*/

ManipulatorCommand youbot::MTaskFreeDrive::GetCommand(const JointsState& new_state)
{
	/*
	std::ofstream logFile;
	logFile.open("nyomatek_log_5.txt", std::ios::app);
	auto kJJJ= std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	logFile << std::ctime(&kJJJ);
	log(Log::info, std::ctime(&kJJJ));
	for (size_t i = 0; i < 5; i++)
	{
		std::string out = "A #" + std::to_string(i) + " csuklo nyomateka: " + std::to_string(new_state.joint[i].tau.value);
		log(Log::info, out);
		logFile << out<< std::endl;
	}
	logFile << std::endl;
	logFile << std::endl;
	logFile <<"%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
	logFile.close();

	return ManipulatorCommand(new BLDCCommand(BLDCCommand::JOINT_VELOCITY,0));
	*/

	Eigen::VectorXd l(5);//li;
	l << 0.0, 0.155, 0.135, 0.171, 0.0;

	Eigen::VectorXd m(5);//link0, link1, link3...;
	//0.769  0.906
	m << 1.39, 1.318, 0.821, 0.769, 0.879;

	//Eigen::VectorXd tau(5);

	//tau[0] = 0;
	//tau[1]=m[1]

	//tau[0] = 0;
	//new_state.joint[1].status
	Eigen::VectorXd M1(2, 2);
	Eigen::VectorXd M1001(3, 3);
	M1001 << 1.39, 0, 0,
		0, 1.39, 0,
		0, 0, 1.39;
	//M1[0, 0] << M1001;
	
	// Write to the file
	std::ofstream logFile;
	logFile.open("nyomatek_log_5.csv", std::ios::app);
	std::string logString = "";


	ManipulatorCommand cmd;
	cmd.commands[0] = BLDCCommand(BLDCCommand::JOINT_VELOCITY, 0);
	cmd.commands[1] = BLDCCommand(BLDCCommand::JOINT_VELOCITY, 0);
	cmd.commands[2] = BLDCCommand(BLDCCommand::JOINT_VELOCITY, 0);
	
	cmd.commands[4] = BLDCCommand(BLDCCommand::JOINT_VELOCITY, 0);
	

	double q3 = (new_state.joint[3].q.value);
	double tau3 = m[3] * gravitational_acceleration * l[3]/2 * std::sin((q3)*M_PI);

	double q2 = (new_state.joint[2].q.value);
	//double tau2 = m[3] * gravitational_acceleration * (l[3] / 2 * std::cos((0.5 - q3) * M_PI) + l[2] * std::cos((0.5 - q2) * M_PI)) +
		//m[2] * gravitational_acceleration * l[2] / 2 * std::cos((0.5 - q2) * M_PI);


	
	//log(Log::info, "#######    TAU 2   ######### :   " + std::to_string(tau2));
	log(Log::info, "#######    TAU 3   ######### :   " + std::to_string(tau3));
	//cmd.commands[2] = BLDCCommand(BLDCCommand::JOINT_TORQUE, tau2);
	cmd.commands[3] = BLDCCommand(BLDCCommand::JOINT_TORQUE, tau3);
	//cmd.commands[3] = BLDCCommand(BLDCCommand::JOINT_VELOCITY, 0);

	//tau << 0, 0, 0, 0, 0;
	//return ManipulatorCommand(BLDCCommand::JOINT_TORQUE, tau);

	return cmd;

	logString += std::to_string(new_state.joint[3].q.value) + ",";
	logString += std::to_string(new_state.joint[3].dq.value) + ",";
	logString += std::to_string(new_state.joint[3].tau.value) + ",";
	logString += std::to_string(tau3) + ",\n";
	logFile << logString;
	logFile.close();
}

bool youbot::MTaskFreeDrive::_taskFinished() const
{
	return false;
}
