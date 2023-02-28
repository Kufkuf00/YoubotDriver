#include "DummyTask.hpp"

using namespace youbot;

youbot::DummyTask::DummyTask()
{
	lastDirChanged= std::chrono::steady_clock::now();
	start= std::chrono::steady_clock::now();
	counter = 0;
	//std::chrono::milliseconds
}

bool DummyTask::_taskFinished()const {
	return false;
}

ManipulatorTask::TaskType DummyTask::GetType() const {
	return ManipulatorTask::DUMMY_TASK;
}

ManipulatorCommand DummyTask::GetCommand(const JointsState& new_state) {
	Eigen::VectorXd tau(5);
	
	//SLEEP_SEC(2);
	//auto h = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now()-lastDirChanged).count();
	
	auto deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - (counter * std::chrono::milliseconds(4000) + start)).count();
	if (deltaTime<= 1000)
	{
		tau << 0, 0, 0, 0, 0;
		return ManipulatorCommand(ManipulatorCommand::JOINT_VELOCITY, tau);
	}
	else if (deltaTime<=2000)
	{
		tau << 0.1, 0.1, 0.1, 0.1, 0.1;
		return ManipulatorCommand(ManipulatorCommand::JOINT_VELOCITY, tau);
	}
	else if(deltaTime<=3000)
	{
		tau << 0, 0, 0, 0, 0;
		return ManipulatorCommand(ManipulatorCommand::JOINT_VELOCITY, tau);
	}
	else if (deltaTime<=4000)
	{
		tau << -0.1, -0.1, -0.1, -0.1, -0.1;		
		return ManipulatorCommand(ManipulatorCommand::JOINT_VELOCITY, tau);
	}
	else// if(deltaTime<=5000)
	{
		counter++;// = std::chrono::steady_clock::now();
		tau << 0, 0, 0, 0, 0;
		return ManipulatorCommand(ManipulatorCommand::JOINT_VELOCITY, tau);
	}

	
	
}
